package app_test

import (
	"context"
	"io"
	"log"
	"log/slog"
	"math"
	"net"
	"testing"
	"time"

	"github.com/simonvetter/modbus"

	"virtualdriverapp/modbus-flow-adapter/internal/app"
	"virtualdriverapp/modbus-flow-adapter/internal/config"
)

func TestEndToEndTCPForwarding(t *testing.T) {
	inputAddress := reserveAddress(t)
	outputAddress := reserveAddress(t)
	targetHandler := newTargetHandler(3000)
	targetServer, err := modbus.NewServer(&modbus.ServerConfiguration{
		URL:        "tcp://" + outputAddress,
		Timeout:    5 * time.Second,
		MaxClients: 2,
		Logger:     log.New(io.Discard, "", 0),
	}, targetHandler)
	if err != nil {
		t.Fatal(err)
	}
	if err := targetServer.Start(); err != nil {
		t.Fatal(err)
	}
	defer targetServer.Stop()

	cfg := config.Default()
	cfg.App.LogEachUpdate = false
	cfg.Input.Listen = inputAddress
	cfg.Output.Endpoint = outputAddress
	cfg.Output.Timeout.Duration = time.Second
	ctx, cancel := context.WithCancel(context.Background())
	appDone := make(chan error, 1)
	go func() {
		appDone <- app.Run(ctx, cfg, slog.New(slog.NewTextHandler(io.Discard, nil)))
	}()
	defer func() {
		cancel()
		select {
		case err := <-appDone:
			if err != nil {
				t.Errorf("adapter shutdown: %v", err)
			}
		case <-time.After(2 * time.Second):
			t.Error("adapter did not stop")
		}
	}()

	sourceClient, err := modbus.NewClient(&modbus.ClientConfiguration{
		URL:     "tcp://" + inputAddress,
		Timeout: time.Second,
		Logger:  log.New(io.Discard, "", 0),
	})
	if err != nil {
		t.Fatal(err)
	}
	if err := sourceClient.SetUnitId(1); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(2 * time.Second)
	for {
		err = sourceClient.Open()
		if err == nil {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("connect to adapter: %v", err)
		}
		time.Sleep(10 * time.Millisecond)
	}
	defer sourceClient.Close()

	// Existing simulator layout: pressure/flow pairs at raw addresses 0..7.
	if err := sourceClient.WriteRegisters(0, []uint16{1000, 1250, 2000, 2500, 3000, 3750, 4000, 5000}); err != nil {
		t.Fatal(err)
	}

	waitForWrites(t, targetHandler.writes, 2)
	assertFloat32(t, targetHandler.registers, 1004, 12.5)
	assertFloat32(t, targetHandler.registers, 1006, 25)
	assertFloat32(t, targetHandler.registers, 2004, 37.5)
	assertFloat32(t, targetHandler.registers, 2006, 50)
}

type targetHandler struct {
	registers []uint16
	writes    chan struct{}
}

func newTargetHandler(count int) *targetHandler {
	return &targetHandler{registers: make([]uint16, count), writes: make(chan struct{}, 8)}
}

func (h *targetHandler) HandleCoils(_ *modbus.CoilsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *targetHandler) HandleDiscreteInputs(_ *modbus.DiscreteInputsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *targetHandler) HandleInputRegisters(_ *modbus.InputRegistersRequest) ([]uint16, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *targetHandler) HandleHoldingRegisters(req *modbus.HoldingRegistersRequest) ([]uint16, error) {
	start := int(req.Addr)
	end := start + int(req.Quantity)
	if req.UnitId != 1 || start < 0 || end > len(h.registers) {
		return nil, modbus.ErrIllegalDataAddress
	}
	if req.IsWrite {
		copy(h.registers[start:end], req.Args)
		h.writes <- struct{}{}
		return nil, nil
	}
	return append([]uint16(nil), h.registers[start:end]...), nil
}

func reserveAddress(t *testing.T) string {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	address := listener.Addr().String()
	if err := listener.Close(); err != nil {
		t.Fatal(err)
	}
	return address
}

func waitForWrites(t *testing.T, writes <-chan struct{}, count int) {
	t.Helper()
	deadline := time.After(2 * time.Second)
	for index := 0; index < count; index++ {
		select {
		case <-writes:
		case <-deadline:
			t.Fatalf("received %d/%d output writes", index, count)
		}
	}
}

func assertFloat32(t *testing.T, registers []uint16, address int, want float32) {
	t.Helper()
	bits := uint32(registers[address])<<16 | uint32(registers[address+1])
	got := math.Float32frombits(bits)
	if got != want {
		t.Fatalf("registers %d-%d = %v, want %v", address, address+1, got, want)
	}
}
