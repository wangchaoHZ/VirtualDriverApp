package input

import (
	"testing"

	"github.com/simonvetter/modbus"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
	"virtualdriverapp/modbus-flow-adapter/internal/model"
)

func TestHoldingRegisterWritePublishesFourFlows(t *testing.T) {
	cfg := config.Default().Input
	updates := make(chan model.FlowSnapshot, 1)
	handler := newRegisterHandler(cfg, func(snapshot model.FlowSnapshot) { updates <- snapshot })

	values := []uint16{1234, 4567, 2345, 5678, 3456, 6789, 4567, 7890}
	_, err := handler.HandleHoldingRegisters(&modbus.HoldingRegistersRequest{
		UnitId:   uint8(cfg.UnitID),
		Addr:     0,
		Quantity: 8,
		IsWrite:  true,
		Args:     values,
	})
	if err != nil {
		t.Fatal(err)
	}

	snapshot := <-updates
	if snapshot.Sequence != 1 ||
		snapshot.S1PositiveFlowM3H != 45.67 ||
		snapshot.S1NegativeFlowM3H != 56.78 ||
		snapshot.S2PositiveFlowM3H != 67.89 ||
		snapshot.S2NegativeFlowM3H != 78.90 {
		t.Fatalf("unexpected snapshot: %+v", snapshot)
	}

	read, err := handler.HandleHoldingRegisters(&modbus.HoldingRegistersRequest{
		UnitId:   uint8(cfg.UnitID),
		Addr:     0,
		Quantity: 8,
	})
	if err != nil {
		t.Fatal(err)
	}
	for index := range values {
		if read[index] != values[index] {
			t.Fatalf("register %d = %d, want %d", index, read[index], values[index])
		}
	}
}

func TestNoPublishUntilAllFlowsSeen(t *testing.T) {
	cfg := config.Default().Input
	updates := make(chan model.FlowSnapshot, 1)
	handler := newRegisterHandler(cfg, func(snapshot model.FlowSnapshot) { updates <- snapshot })

	for _, address := range []uint16{1, 3, 5} {
		_, err := handler.HandleHoldingRegisters(&modbus.HoldingRegistersRequest{
			UnitId: uint8(cfg.UnitID), Addr: address, Quantity: 1, IsWrite: true, Args: []uint16{100},
		})
		if err != nil {
			t.Fatal(err)
		}
	}
	select {
	case snapshot := <-updates:
		t.Fatalf("published too early: %+v", snapshot)
	default:
	}

	_, err := handler.HandleHoldingRegisters(&modbus.HoldingRegistersRequest{
		UnitId: uint8(cfg.UnitID), Addr: 7, Quantity: 1, IsWrite: true, Args: []uint16{100},
	})
	if err != nil {
		t.Fatal(err)
	}
	select {
	case <-updates:
	default:
		t.Fatal("expected a complete snapshot")
	}
}
