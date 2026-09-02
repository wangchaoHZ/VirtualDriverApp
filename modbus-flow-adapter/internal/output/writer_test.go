package output

import (
	"io"
	"log/slog"
	"math"
	"testing"
	"time"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
	"virtualdriverapp/modbus-flow-adapter/internal/model"
)

type recordedWrite struct {
	address uint16
	values  []uint16
}

type fakeClient struct {
	writes []recordedWrite
}

func (f *fakeClient) Open() error  { return nil }
func (f *fakeClient) Close() error { return nil }
func (f *fakeClient) WriteRegisters(address uint16, values []uint16) error {
	f.writes = append(f.writes, recordedWrite{address: address, values: append([]uint16(nil), values...)})
	return nil
}

func TestWriteSnapshotUsesTwoFC16Blocks(t *testing.T) {
	cfg := config.Default().Output
	writer, err := NewWriter(cfg, discardLogger())
	if err != nil {
		t.Fatal(err)
	}
	client := &fakeClient{}
	writer.client = client

	snapshot := model.FlowSnapshot{
		ReceivedAt:        time.Now(),
		S1PositiveFlowM3H: 12.5,
		S1NegativeFlowM3H: 25,
		S2PositiveFlowM3H: 37.5,
		S2NegativeFlowM3H: 50,
	}
	if err := writer.writeSnapshot(snapshot); err != nil {
		t.Fatal(err)
	}
	if len(client.writes) != 2 {
		t.Fatalf("got %d writes, want 2", len(client.writes))
	}
	if client.writes[0].address != 1004 || client.writes[1].address != 2004 {
		t.Fatalf("unexpected addresses: %+v", client.writes)
	}
	assertFloatPair(t, client.writes[0].values, 12.5, 25)
	assertFloatPair(t, client.writes[1].values, 37.5, 50)
}

func assertFloatPair(t *testing.T, got []uint16, first, second float32) {
	t.Helper()
	firstBits := math.Float32bits(first)
	secondBits := math.Float32bits(second)
	want := []uint16{uint16(firstBits >> 16), uint16(firstBits), uint16(secondBits >> 16), uint16(secondBits)}
	if len(got) != len(want) {
		t.Fatalf("got %d registers, want %d", len(got), len(want))
	}
	for index := range want {
		if got[index] != want[index] {
			t.Fatalf("register %d = %#04x, want %#04x", index, got[index], want[index])
		}
	}
}

func discardLogger() *slog.Logger {
	return slog.New(slog.NewTextHandler(io.Discard, nil))
}
