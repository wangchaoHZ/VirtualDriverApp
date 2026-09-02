package model

import "time"

// FlowSnapshot is one complete four-pump flow update received from the
// VirtualDriverApp Modbus TCP output.
type FlowSnapshot struct {
	Sequence          uint64
	ReceivedAt        time.Time
	S1PositiveFlowM3H float64
	S1NegativeFlowM3H float64
	S2PositiveFlowM3H float64
	S2NegativeFlowM3H float64
}
