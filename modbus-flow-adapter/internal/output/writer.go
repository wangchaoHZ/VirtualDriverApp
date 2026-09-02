package output

import (
	"context"
	"fmt"
	"io"
	"log"
	"log/slog"
	"time"

	"github.com/simonvetter/modbus"

	"virtualdriverapp/modbus-flow-adapter/internal/codec"
	"virtualdriverapp/modbus-flow-adapter/internal/config"
	"virtualdriverapp/modbus-flow-adapter/internal/model"
)

type modbusClient interface {
	Open() error
	Close() error
	WriteRegisters(addr uint16, values []uint16) error
}

type clientFactory func() (modbusClient, error)

type Writer struct {
	cfg           config.OutputConfig
	logger        *slog.Logger
	factory       clientFactory
	client        modbusClient
	s1FlowAddress uint16
	s2FlowAddress uint16
}

func NewWriter(cfg config.OutputConfig, logger *slog.Logger) (*Writer, error) {
	s1Address, err := config.ProtocolAddress(cfg.Mappings.S1.PositiveFlowM3H, cfg.RegisterNotation)
	if err != nil {
		return nil, err
	}
	s2Address, err := config.ProtocolAddress(cfg.Mappings.S2.PositiveFlowM3H, cfg.RegisterNotation)
	if err != nil {
		return nil, err
	}

	writer := &Writer{
		cfg:           cfg,
		logger:        logger,
		s1FlowAddress: s1Address,
		s2FlowAddress: s2Address,
	}
	writer.factory = writer.newClient
	return writer, nil
}

func (w *Writer) Run(ctx context.Context, updates <-chan model.FlowSnapshot, logEachUpdate bool) {
	defer w.disconnect()

	for {
		var pending model.FlowSnapshot
		select {
		case <-ctx.Done():
			return
		case pending = <-updates:
		}
		pending = drainLatest(pending, updates)

		backoff := w.cfg.ReconnectInitial.Duration
		for {
			if ctx.Err() != nil {
				return
			}
			if w.client == nil {
				if err := w.connect(); err != nil {
					w.logger.Warn("连接输出 Modbus TCP 设备失败，准备重试", "endpoint", w.cfg.Endpoint, "error", err, "retry_in", backoff)
					var ok bool
					pending, ok = waitForRetry(ctx, updates, pending, backoff)
					if !ok {
						return
					}
					backoff = minDuration(backoff*2, w.cfg.ReconnectMax.Duration)
					continue
				}
			}

			pending = drainLatest(pending, updates)
			if err := w.writeSnapshot(pending); err != nil {
				w.logger.Warn("输出流量失败，将保留最新值并重连", "endpoint", w.cfg.Endpoint, "sequence", pending.Sequence, "error", err)
				w.disconnect()
				var ok bool
				pending, ok = waitForRetry(ctx, updates, pending, backoff)
				if !ok {
					return
				}
				backoff = minDuration(backoff*2, w.cfg.ReconnectMax.Duration)
				continue
			}

			if logEachUpdate {
				w.logger.Info("流量已转发",
					"sequence", pending.Sequence,
					"s1_positive_m3h", pending.S1PositiveFlowM3H,
					"s1_negative_m3h", pending.S1NegativeFlowM3H,
					"s2_positive_m3h", pending.S2PositiveFlowM3H,
					"s2_negative_m3h", pending.S2NegativeFlowM3H,
					"latency", time.Since(pending.ReceivedAt).Round(time.Microsecond))
			}
			backoff = w.cfg.ReconnectInitial.Duration
			break
		}
	}
}

func (w *Writer) newClient() (modbusClient, error) {
	client, err := modbus.NewClient(&modbus.ClientConfiguration{
		URL:     "tcp://" + w.cfg.Endpoint,
		Timeout: w.cfg.Timeout.Duration,
		Logger:  log.New(io.Discard, "", 0),
	})
	if err != nil {
		return nil, fmt.Errorf("创建客户端: %w", err)
	}
	if err := client.SetUnitId(uint8(w.cfg.UnitID)); err != nil {
		return nil, fmt.Errorf("设置 Unit ID: %w", err)
	}
	return client, nil
}

func (w *Writer) connect() error {
	client, err := w.factory()
	if err != nil {
		return err
	}
	if err := client.Open(); err != nil {
		_ = client.Close()
		return err
	}
	w.client = client
	w.logger.Info("已连接输出 Modbus TCP 设备", "endpoint", w.cfg.Endpoint, "unit_id", w.cfg.UnitID)
	return nil
}

func (w *Writer) disconnect() {
	if w.client == nil {
		return
	}
	if err := w.client.Close(); err != nil {
		w.logger.Debug("关闭输出 Modbus TCP 连接时发生错误", "error", err)
	}
	w.client = nil
}

func (w *Writer) writeSnapshot(snapshot model.FlowSnapshot) error {
	s1Registers, err := codec.EncodePair(snapshot.S1PositiveFlowM3H, snapshot.S1NegativeFlowM3H, w.cfg.Encoding)
	if err != nil {
		return fmt.Errorf("编码 S1 流量: %w", err)
	}
	s2Registers, err := codec.EncodePair(snapshot.S2PositiveFlowM3H, snapshot.S2NegativeFlowM3H, w.cfg.Encoding)
	if err != nil {
		return fmt.Errorf("编码 S2 流量: %w", err)
	}

	// WriteRegisters always uses Modbus function code 16. One contiguous write is
	// used per subsystem so the positive/negative values change together.
	if err := w.client.WriteRegisters(w.s1FlowAddress, s1Registers); err != nil {
		return fmt.Errorf("FC16 写入 S1 地址 %d: %w", w.s1FlowAddress, err)
	}
	if err := w.client.WriteRegisters(w.s2FlowAddress, s2Registers); err != nil {
		return fmt.Errorf("FC16 写入 S2 地址 %d: %w", w.s2FlowAddress, err)
	}
	return nil
}

func drainLatest(current model.FlowSnapshot, updates <-chan model.FlowSnapshot) model.FlowSnapshot {
	for {
		select {
		case current = <-updates:
		default:
			return current
		}
	}
}

func waitForRetry(ctx context.Context, updates <-chan model.FlowSnapshot, current model.FlowSnapshot, delay time.Duration) (model.FlowSnapshot, bool) {
	timer := time.NewTimer(delay)
	defer timer.Stop()
	for {
		select {
		case <-ctx.Done():
			return current, false
		case current = <-updates:
			current = drainLatest(current, updates)
		case <-timer.C:
			return current, true
		}
	}
}

func minDuration(left, right time.Duration) time.Duration {
	if left < right {
		return left
	}
	return right
}
