package config

import "testing"

func TestDefaultConfigurationIsValid(t *testing.T) {
	if err := Default().Validate(); err != nil {
		t.Fatal(err)
	}
}

func TestProtocolAddress(t *testing.T) {
	tests := []struct {
		configured int
		notation   string
		want       uint16
	}{
		{41005, Notation4XOneBased, 1004},
		{42007, Notation4XOneBased, 2006},
		{1004, NotationRaw, 1004},
	}
	for _, test := range tests {
		got, err := ProtocolAddress(test.configured, test.notation)
		if err != nil {
			t.Fatal(err)
		}
		if got != test.want {
			t.Fatalf("ProtocolAddress(%d, %q) = %d, want %d", test.configured, test.notation, got, test.want)
		}
	}
}

func TestValidateRejectsNonContiguousFlowPair(t *testing.T) {
	cfg := Default()
	cfg.Output.Mappings.S1.NegativeFlowM3H = 41008
	if err := cfg.Validate(); err == nil {
		t.Fatal("expected non-contiguous mapping to fail")
	}
}
