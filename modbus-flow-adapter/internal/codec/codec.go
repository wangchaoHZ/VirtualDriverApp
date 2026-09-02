package codec

import (
	"fmt"
	"math"

	"virtualdriverapp/modbus-flow-adapter/internal/config"
)

// Encode32 encodes one engineering value into two raw Modbus registers.
func Encode32(value float64, encoding config.EncodingConfig) ([2]uint16, error) {
	if math.IsNaN(value) || math.IsInf(value, 0) {
		return [2]uint16{}, fmt.Errorf("数值 %v 不是有限数", value)
	}

	var bits uint32
	switch encoding.Type {
	case "float32":
		converted := float32(value)
		if math.IsInf(float64(converted), 0) {
			return [2]uint16{}, fmt.Errorf("数值 %v 超出 float32 范围", value)
		}
		bits = math.Float32bits(converted)
	case "uint32_scaled":
		scaled := math.Round(value * encoding.Scale)
		if scaled < 0 || scaled > math.MaxUint32 {
			return [2]uint16{}, fmt.Errorf("数值 %v 按比例 %v 编码后超出 uint32 范围", value, encoding.Scale)
		}
		bits = uint32(scaled)
	case "int32_scaled":
		scaled := math.Round(value * encoding.Scale)
		if scaled < math.MinInt32 || scaled > math.MaxInt32 {
			return [2]uint16{}, fmt.Errorf("数值 %v 按比例 %v 编码后超出 int32 范围", value, encoding.Scale)
		}
		bits = uint32(int32(scaled))
	default:
		return [2]uint16{}, fmt.Errorf("不支持的编码类型 %q", encoding.Type)
	}

	high := uint16(bits >> 16)
	low := uint16(bits)
	if encoding.ByteOrder == "little" {
		high = swapBytes(high)
		low = swapBytes(low)
	} else if encoding.ByteOrder != "big" {
		return [2]uint16{}, fmt.Errorf("不支持的字节序 %q", encoding.ByteOrder)
	}

	if encoding.WordOrder == "low_first" {
		return [2]uint16{low, high}, nil
	}
	if encoding.WordOrder != "high_first" {
		return [2]uint16{}, fmt.Errorf("不支持的字序 %q", encoding.WordOrder)
	}
	return [2]uint16{high, low}, nil
}

func EncodePair(first, second float64, encoding config.EncodingConfig) ([]uint16, error) {
	firstRegisters, err := Encode32(first, encoding)
	if err != nil {
		return nil, err
	}
	secondRegisters, err := Encode32(second, encoding)
	if err != nil {
		return nil, err
	}
	return []uint16{
		firstRegisters[0], firstRegisters[1],
		secondRegisters[0], secondRegisters[1],
	}, nil
}

func swapBytes(value uint16) uint16 {
	return value<<8 | value>>8
}
