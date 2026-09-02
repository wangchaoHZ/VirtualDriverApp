package config

import (
	"errors"
	"fmt"
	"io"
	"net"
	"os"
	"strconv"
	"strings"
	"time"

	"gopkg.in/yaml.v3"
)

const (
	Notation4XOneBased = "4xxxx_one_based"
	NotationRaw        = "raw_zero_based"
)

type Duration struct {
	time.Duration
}

func (d *Duration) UnmarshalYAML(value *yaml.Node) error {
	parsed, err := time.ParseDuration(strings.TrimSpace(value.Value))
	if err != nil {
		return fmt.Errorf("无效时长 %q: %w", value.Value, err)
	}
	d.Duration = parsed
	return nil
}

type Config struct {
	App    AppConfig    `yaml:"app"`
	Input  InputConfig  `yaml:"input"`
	Output OutputConfig `yaml:"output"`
}

type AppConfig struct {
	LogLevel      string `yaml:"log_level"`
	LogEachUpdate bool   `yaml:"log_each_update"`
	QueueCapacity int    `yaml:"queue_capacity"`
}

type InputConfig struct {
	Listen               string           `yaml:"listen"`
	UnitID               int              `yaml:"unit_id"`
	IdleTimeout          Duration         `yaml:"idle_timeout"`
	MaxClients           uint             `yaml:"max_clients"`
	HoldingRegisterCount int              `yaml:"holding_register_count"`
	FlowScale            float64          `yaml:"flow_scale"`
	Registers            InputRegisterMap `yaml:"registers"`
}

type InputRegisterMap struct {
	P1Flow int `yaml:"p1_flow"`
	N1Flow int `yaml:"n1_flow"`
	P2Flow int `yaml:"p2_flow"`
	N2Flow int `yaml:"n2_flow"`
}

type OutputConfig struct {
	Endpoint         string            `yaml:"endpoint"`
	UnitID           int               `yaml:"unit_id"`
	Timeout          Duration          `yaml:"timeout"`
	ReconnectInitial Duration          `yaml:"reconnect_initial"`
	ReconnectMax     Duration          `yaml:"reconnect_max"`
	RegisterNotation string            `yaml:"register_notation"`
	Encoding         EncodingConfig    `yaml:"encoding"`
	Mappings         OutputRegisterMap `yaml:"mappings"`
}

type EncodingConfig struct {
	Type      string  `yaml:"type"`
	Scale     float64 `yaml:"scale"`
	ByteOrder string  `yaml:"byte_order"`
	WordOrder string  `yaml:"word_order"`
}

type OutputRegisterMap struct {
	S1 SubsystemRegisterMap `yaml:"s1"`
	S2 SubsystemRegisterMap `yaml:"s2"`
}

type SubsystemRegisterMap struct {
	Mode            int `yaml:"mode"`
	CurrentA        int `yaml:"current_a"`
	PositiveFlowM3H int `yaml:"positive_flow_m3h"`
	NegativeFlowM3H int `yaml:"negative_flow_m3h"`
}

func Default() Config {
	return Config{
		App: AppConfig{
			LogLevel:      "info",
			LogEachUpdate: true,
			QueueCapacity: 1,
		},
		Input: InputConfig{
			Listen:               "0.0.0.0:1502",
			UnitID:               1,
			IdleTimeout:          Duration{60 * time.Second},
			MaxClients:           8,
			HoldingRegisterCount: 8,
			FlowScale:            100,
			Registers: InputRegisterMap{
				P1Flow: 1,
				N1Flow: 3,
				P2Flow: 5,
				N2Flow: 7,
			},
		},
		Output: OutputConfig{
			Endpoint:         "127.0.0.1:502",
			UnitID:           1,
			Timeout:          Duration{500 * time.Millisecond},
			ReconnectInitial: Duration{100 * time.Millisecond},
			ReconnectMax:     Duration{2 * time.Second},
			RegisterNotation: Notation4XOneBased,
			Encoding: EncodingConfig{
				Type:      "float32",
				Scale:     100,
				ByteOrder: "big",
				WordOrder: "high_first",
			},
			Mappings: OutputRegisterMap{
				S1: SubsystemRegisterMap{41001, 41003, 41005, 41007},
				S2: SubsystemRegisterMap{42001, 42003, 42005, 42007},
			},
		},
	}
}

func Load(path string) (Config, error) {
	cfg := Default()

	file, err := os.Open(path)
	if err != nil {
		return Config{}, fmt.Errorf("打开配置文件 %q: %w", path, err)
	}
	defer file.Close()

	decoder := yaml.NewDecoder(file)
	decoder.KnownFields(true)
	if err := decoder.Decode(&cfg); err != nil {
		return Config{}, fmt.Errorf("解析配置文件 %q: %w", path, err)
	}

	var extra any
	if err := decoder.Decode(&extra); err != io.EOF {
		if err == nil {
			return Config{}, errors.New("配置文件只能包含一个 YAML 文档")
		}
		return Config{}, fmt.Errorf("解析配置文件尾部: %w", err)
	}

	if err := cfg.Validate(); err != nil {
		return Config{}, err
	}
	return cfg, nil
}

func (c Config) Validate() error {
	var problems []string
	if !isLogLevel(c.App.LogLevel) {
		problems = append(problems, "app.log_level 必须是 debug、info、warn 或 error")
	}
	if c.App.QueueCapacity < 1 || c.App.QueueCapacity > 1024 {
		problems = append(problems, "app.queue_capacity 必须在 1..1024 之间")
	}

	if err := validateEndpoint(c.Input.Listen); err != nil {
		problems = append(problems, "input.listen: "+err.Error())
	}
	if c.Input.UnitID < 0 || c.Input.UnitID > 255 {
		problems = append(problems, "input.unit_id 必须在 0..255 之间")
	}
	if c.Input.IdleTimeout.Duration <= 0 {
		problems = append(problems, "input.idle_timeout 必须大于 0")
	}
	if c.Input.MaxClients == 0 {
		problems = append(problems, "input.max_clients 必须大于 0")
	}
	if c.Input.HoldingRegisterCount < 8 || c.Input.HoldingRegisterCount > 65536 {
		problems = append(problems, "input.holding_register_count 必须在 8..65536 之间")
	}
	if c.Input.FlowScale <= 0 {
		problems = append(problems, "input.flow_scale 必须大于 0")
	}
	inputAddresses := []int{
		c.Input.Registers.P1Flow,
		c.Input.Registers.N1Flow,
		c.Input.Registers.P2Flow,
		c.Input.Registers.N2Flow,
	}
	seen := make(map[int]struct{}, len(inputAddresses))
	for _, address := range inputAddresses {
		if address < 0 || address >= c.Input.HoldingRegisterCount {
			problems = append(problems, fmt.Sprintf("输入流量寄存器地址 %d 超出 0..%d", address, c.Input.HoldingRegisterCount-1))
		}
		if _, exists := seen[address]; exists {
			problems = append(problems, fmt.Sprintf("输入流量寄存器地址 %d 重复", address))
		}
		seen[address] = struct{}{}
	}

	if err := validateEndpoint(c.Output.Endpoint); err != nil {
		problems = append(problems, "output.endpoint: "+err.Error())
	}
	if c.Output.UnitID < 0 || c.Output.UnitID > 255 {
		problems = append(problems, "output.unit_id 必须在 0..255 之间")
	}
	if c.Output.Timeout.Duration <= 0 {
		problems = append(problems, "output.timeout 必须大于 0")
	}
	if c.Output.ReconnectInitial.Duration <= 0 {
		problems = append(problems, "output.reconnect_initial 必须大于 0")
	}
	if c.Output.ReconnectMax.Duration <= 0 {
		problems = append(problems, "output.reconnect_max 必须大于 0")
	}
	if c.Output.ReconnectMax.Duration < c.Output.ReconnectInitial.Duration {
		problems = append(problems, "output.reconnect_max 不能小于 reconnect_initial")
	}
	if c.Output.RegisterNotation != Notation4XOneBased && c.Output.RegisterNotation != NotationRaw {
		problems = append(problems, "output.register_notation 必须是 4xxxx_one_based 或 raw_zero_based")
	}
	if !isEncodingType(c.Output.Encoding.Type) {
		problems = append(problems, "output.encoding.type 必须是 float32、uint32_scaled 或 int32_scaled")
	}
	if c.Output.Encoding.Type != "float32" && c.Output.Encoding.Scale <= 0 {
		problems = append(problems, "output.encoding.scale 在定点整数编码下必须大于 0")
	}
	if c.Output.Encoding.ByteOrder != "big" && c.Output.Encoding.ByteOrder != "little" {
		problems = append(problems, "output.encoding.byte_order 必须是 big 或 little")
	}
	if c.Output.Encoding.WordOrder != "high_first" && c.Output.Encoding.WordOrder != "low_first" {
		problems = append(problems, "output.encoding.word_order 必须是 high_first 或 low_first")
	}

	for name, mapping := range map[string]SubsystemRegisterMap{"s1": c.Output.Mappings.S1, "s2": c.Output.Mappings.S2} {
		for field, address := range map[string]int{
			"mode": mapping.Mode, "current_a": mapping.CurrentA,
			"positive_flow_m3h": mapping.PositiveFlowM3H,
			"negative_flow_m3h": mapping.NegativeFlowM3H,
		} {
			protocolAddress, err := ProtocolAddress(address, c.Output.RegisterNotation)
			if err != nil {
				problems = append(problems, fmt.Sprintf("output.mappings.%s.%s: %v", name, field, err))
			} else if protocolAddress == 65535 {
				problems = append(problems, fmt.Sprintf("output.mappings.%s.%s 没有足够空间容纳两个寄存器", name, field))
			}
		}
		positive, positiveErr := ProtocolAddress(mapping.PositiveFlowM3H, c.Output.RegisterNotation)
		negative, negativeErr := ProtocolAddress(mapping.NegativeFlowM3H, c.Output.RegisterNotation)
		if positiveErr == nil && negativeErr == nil && int(negative) != int(positive)+2 {
			problems = append(problems, fmt.Sprintf("output.mappings.%s 的负极流量地址必须紧接正极流量的两个寄存器之后", name))
		}
	}

	if len(problems) > 0 {
		return errors.New("配置无效:\n- " + strings.Join(problems, "\n- "))
	}
	return nil
}

func ProtocolAddress(configured int, notation string) (uint16, error) {
	address := configured
	switch notation {
	case Notation4XOneBased:
		if configured < 40001 || configured > 105536 {
			return 0, fmt.Errorf("4xxxx 地址 %d 超出 40001..105536", configured)
		}
		address = configured - 40001
	case NotationRaw:
		// Already zero-based.
	default:
		return 0, fmt.Errorf("未知地址格式 %q", notation)
	}
	if address < 0 || address > 65535 {
		return 0, fmt.Errorf("协议地址 %d 超出 0..65535", address)
	}
	return uint16(address), nil
}

func validateEndpoint(endpoint string) error {
	if strings.TrimSpace(endpoint) != endpoint || endpoint == "" {
		return fmt.Errorf("必须是 host:port，且不能包含首尾空格")
	}
	host, port, err := net.SplitHostPort(endpoint)
	if err != nil {
		return fmt.Errorf("必须是 host:port（IPv6 地址需使用方括号）: %w", err)
	}
	if host == "" {
		return errors.New("IP/主机名不能为空")
	}
	portNumber, err := strconv.Atoi(port)
	if err != nil || portNumber < 1 || portNumber > 65535 {
		return errors.New("端口必须是 1..65535 的数字")
	}
	return nil
}

func isLogLevel(level string) bool {
	switch strings.ToLower(level) {
	case "debug", "info", "warn", "error":
		return true
	default:
		return false
	}
}

func isEncodingType(encodingType string) bool {
	switch encodingType {
	case "float32", "uint32_scaled", "int32_scaled":
		return true
	default:
		return false
	}
}
