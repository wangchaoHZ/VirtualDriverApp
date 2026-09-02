package input

import (
	"fmt"
	"io"
	"log"
	"log/slog"
	"sync"
	"time"

	"github.com/simonvetter/modbus"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
	"virtualdriverapp/modbus-flow-adapter/internal/model"
)

type SnapshotHandler func(model.FlowSnapshot)

type Server struct {
	server  *modbus.ModbusServer
	handler *registerHandler
	logger  *slog.Logger
}

func NewServer(cfg config.InputConfig, logger *slog.Logger, onSnapshot SnapshotHandler) (*Server, error) {
	handler := newRegisterHandler(cfg, onSnapshot)
	server, err := modbus.NewServer(&modbus.ServerConfiguration{
		URL:        "tcp://" + cfg.Listen,
		Timeout:    cfg.IdleTimeout.Duration,
		MaxClients: cfg.MaxClients,
		Logger:     log.New(io.Discard, "", 0),
	}, handler)
	if err != nil {
		return nil, fmt.Errorf("创建输入 Modbus TCP 从站: %w", err)
	}
	return &Server{server: server, handler: handler, logger: logger}, nil
}

func (s *Server) Start() error {
	if err := s.server.Start(); err != nil {
		return fmt.Errorf("监听输入 Modbus TCP: %w", err)
	}
	return nil
}

func (s *Server) Stop() error {
	if err := s.server.Stop(); err != nil {
		return fmt.Errorf("停止输入 Modbus TCP: %w", err)
	}
	return nil
}

type registerHandler struct {
	mu          sync.RWMutex
	cfg         config.InputConfig
	registers   []uint16
	flowSeen    map[int]bool
	sequence    uint64
	onSnapshot  SnapshotHandler
	flowAddress map[int]struct{}
}

func newRegisterHandler(cfg config.InputConfig, onSnapshot SnapshotHandler) *registerHandler {
	flowAddress := map[int]struct{}{
		cfg.Registers.P1Flow: {},
		cfg.Registers.N1Flow: {},
		cfg.Registers.P2Flow: {},
		cfg.Registers.N2Flow: {},
	}
	return &registerHandler{
		cfg:         cfg,
		registers:   make([]uint16, cfg.HoldingRegisterCount),
		flowSeen:    make(map[int]bool, 4),
		onSnapshot:  onSnapshot,
		flowAddress: flowAddress,
	}
}

func (h *registerHandler) HandleCoils(_ *modbus.CoilsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *registerHandler) HandleDiscreteInputs(_ *modbus.DiscreteInputsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *registerHandler) HandleInputRegisters(_ *modbus.InputRegistersRequest) ([]uint16, error) {
	return nil, modbus.ErrIllegalFunction
}

func (h *registerHandler) HandleHoldingRegisters(req *modbus.HoldingRegistersRequest) ([]uint16, error) {
	if int(req.UnitId) != h.cfg.UnitID {
		return nil, modbus.ErrIllegalDataAddress
	}
	start := int(req.Addr)
	end := start + int(req.Quantity)
	if req.Quantity == 0 || start < 0 || end > len(h.registers) {
		return nil, modbus.ErrIllegalDataAddress
	}

	if !req.IsWrite {
		h.mu.RLock()
		result := append([]uint16(nil), h.registers[start:end]...)
		h.mu.RUnlock()
		return result, nil
	}
	if len(req.Args) != int(req.Quantity) {
		return nil, modbus.ErrIllegalDataValue
	}

	var snapshot *model.FlowSnapshot
	h.mu.Lock()
	copy(h.registers[start:end], req.Args)
	flowTouched := false
	for address := start; address < end; address++ {
		if _, isFlow := h.flowAddress[address]; isFlow {
			h.flowSeen[address] = true
			flowTouched = true
		}
	}
	if flowTouched && len(h.flowSeen) == len(h.flowAddress) {
		h.sequence++
		value := model.FlowSnapshot{
			Sequence:          h.sequence,
			ReceivedAt:        time.Now(),
			S1PositiveFlowM3H: float64(h.registers[h.cfg.Registers.P1Flow]) / h.cfg.FlowScale,
			S1NegativeFlowM3H: float64(h.registers[h.cfg.Registers.N1Flow]) / h.cfg.FlowScale,
			S2PositiveFlowM3H: float64(h.registers[h.cfg.Registers.P2Flow]) / h.cfg.FlowScale,
			S2NegativeFlowM3H: float64(h.registers[h.cfg.Registers.N2Flow]) / h.cfg.FlowScale,
		}
		snapshot = &value
	}
	h.mu.Unlock()

	if snapshot != nil && h.onSnapshot != nil {
		h.onSnapshot(*snapshot)
	}
	return nil, nil
}
