package app

import (
	"context"
	"fmt"
	"log/slog"
	"sync"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
	"virtualdriverapp/modbus-flow-adapter/internal/input"
	"virtualdriverapp/modbus-flow-adapter/internal/model"
	"virtualdriverapp/modbus-flow-adapter/internal/output"
)

func Run(ctx context.Context, cfg config.Config, logger *slog.Logger) error {
	runContext, cancel := context.WithCancel(ctx)
	defer cancel()

	queue := newLatestQueue(cfg.App.QueueCapacity)
	writer, err := output.NewWriter(cfg.Output, logger)
	if err != nil {
		return fmt.Errorf("初始化输出主站: %w", err)
	}

	inputServer, err := input.NewServer(cfg.Input, logger, queue.Offer)
	if err != nil {
		return err
	}

	var writerGroup sync.WaitGroup
	writerGroup.Add(1)
	go func() {
		defer writerGroup.Done()
		writer.Run(runContext, queue.Channel(), cfg.App.LogEachUpdate)
	}()

	if err := inputServer.Start(); err != nil {
		cancel()
		writerGroup.Wait()
		return err
	}
	logger.Info("流量通信适配器已启动",
		"input", cfg.Input.Listen,
		"input_unit_id", cfg.Input.UnitID,
		"output", cfg.Output.Endpoint,
		"output_unit_id", cfg.Output.UnitID,
		"s1_flow_registers", fmt.Sprintf("%d-%d", cfg.Output.Mappings.S1.PositiveFlowM3H, cfg.Output.Mappings.S1.NegativeFlowM3H+1),
		"s2_flow_registers", fmt.Sprintf("%d-%d", cfg.Output.Mappings.S2.PositiveFlowM3H, cfg.Output.Mappings.S2.NegativeFlowM3H+1))

	<-runContext.Done()
	if err := inputServer.Stop(); err != nil {
		logger.Warn("停止输入从站时发生错误", "error", err)
	}
	cancel()
	writerGroup.Wait()
	logger.Info("流量通信适配器已停止")
	return nil
}

type latestQueue struct {
	mu           sync.Mutex
	updates      chan model.FlowSnapshot
	lastSequence uint64
}

func newLatestQueue(capacity int) *latestQueue {
	return &latestQueue{updates: make(chan model.FlowSnapshot, capacity)}
}

func (q *latestQueue) Channel() <-chan model.FlowSnapshot {
	return q.updates
}

// Offer never blocks a Modbus request handler. If the consumer is slower than
// the producer, queued stale snapshots are dropped in favor of the newest one.
func (q *latestQueue) Offer(snapshot model.FlowSnapshot) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if snapshot.Sequence <= q.lastSequence {
		return
	}
	q.lastSequence = snapshot.Sequence

	select {
	case q.updates <- snapshot:
		return
	default:
	}

	for {
		select {
		case <-q.updates:
		default:
			q.updates <- snapshot
			return
		}
	}
}
