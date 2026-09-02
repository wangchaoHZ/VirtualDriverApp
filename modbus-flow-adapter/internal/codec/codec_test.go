package codec

import (
	"math"
	"reflect"
	"testing"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
)

func TestEncodeFloat32Orders(t *testing.T) {
	base := config.EncodingConfig{Type: "float32", ByteOrder: "big", WordOrder: "high_first"}
	bits := math.Float32bits(12.5)
	wantHigh := uint16(bits >> 16)
	wantLow := uint16(bits)

	tests := []struct {
		name      string
		byteOrder string
		wordOrder string
		want      [2]uint16
	}{
		{"big high", "big", "high_first", [2]uint16{wantHigh, wantLow}},
		{"big low", "big", "low_first", [2]uint16{wantLow, wantHigh}},
		{"little high", "little", "high_first", [2]uint16{swapBytes(wantHigh), swapBytes(wantLow)}},
		{"little low", "little", "low_first", [2]uint16{swapBytes(wantLow), swapBytes(wantHigh)}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			encoding := base
			encoding.ByteOrder = test.byteOrder
			encoding.WordOrder = test.wordOrder
			got, err := Encode32(12.5, encoding)
			if err != nil {
				t.Fatal(err)
			}
			if got != test.want {
				t.Fatalf("got %#v, want %#v", got, test.want)
			}
		})
	}
}

func TestEncodeScaledIntegers(t *testing.T) {
	uintEncoding := config.EncodingConfig{Type: "uint32_scaled", Scale: 100, ByteOrder: "big", WordOrder: "high_first"}
	got, err := Encode32(45.67, uintEncoding)
	if err != nil {
		t.Fatal(err)
	}
	if got != [2]uint16{0, 4567} {
		t.Fatalf("uint encoding got %#v", got)
	}

	intEncoding := config.EncodingConfig{Type: "int32_scaled", Scale: 100, ByteOrder: "big", WordOrder: "high_first"}
	got, err = Encode32(-1.25, intEncoding)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(got, [2]uint16{0xffff, 0xff83}) {
		t.Fatalf("int encoding got %#v", got)
	}
}

func TestEncodeRejectsInvalidNumber(t *testing.T) {
	encoding := config.EncodingConfig{Type: "float32", ByteOrder: "big", WordOrder: "high_first"}
	if _, err := Encode32(math.NaN(), encoding); err == nil {
		t.Fatal("expected NaN to be rejected")
	}
}
