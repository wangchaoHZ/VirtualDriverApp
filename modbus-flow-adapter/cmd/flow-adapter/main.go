package main

import (
	"context"
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"

	"virtualdriverapp/modbus-flow-adapter/internal/app"
	"virtualdriverapp/modbus-flow-adapter/internal/config"
)

var version = "dev"

func main() {
	configFlag := flag.String("config", "", "YAML 配置文件路径（默认查找程序同目录或当前目录的 config.yaml）")
	checkOnly := flag.Bool("check", false, "只校验配置，不启动通信")
	showVersion := flag.Bool("version", false, "显示版本")
	flag.Parse()

	if *showVersion {
		fmt.Printf("modbus-flow-adapter %s\n", version)
		return
	}

	configPath, err := resolveConfigPath(*configFlag)
	if err != nil {
		fatal(err)
	}
	cfg, err := config.Load(configPath)
	if err != nil {
		fatal(err)
	}
	if *checkOnly {
		fmt.Printf("配置有效: %s\n", configPath)
		return
	}

	logger := newLogger(cfg.App.LogLevel)
	logger.Info("已加载配置", "path", configPath, "version", version)
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	if err := app.Run(ctx, cfg, logger); err != nil {
		logger.Error("适配器运行失败", "error", err)
		os.Exit(1)
	}
}

func resolveConfigPath(configFlag string) (string, error) {
	if configFlag != "" {
		absolute, err := filepath.Abs(configFlag)
		if err != nil {
			return "", fmt.Errorf("解析配置路径: %w", err)
		}
		return absolute, nil
	}

	executable, err := os.Executable()
	if err == nil {
		nextToExecutable := filepath.Join(filepath.Dir(executable), "config.yaml")
		if _, statErr := os.Stat(nextToExecutable); statErr == nil {
			return nextToExecutable, nil
		}
	}
	current, err := filepath.Abs("config.yaml")
	if err == nil {
		if _, statErr := os.Stat(current); statErr == nil {
			return current, nil
		}
	}
	return "", fmt.Errorf("未找到 config.yaml；请将它放在程序同目录，或使用 -config 指定")
}

func newLogger(configuredLevel string) *slog.Logger {
	level := slog.LevelInfo
	switch strings.ToLower(configuredLevel) {
	case "debug":
		level = slog.LevelDebug
	case "warn":
		level = slog.LevelWarn
	case "error":
		level = slog.LevelError
	}
	return slog.New(slog.NewTextHandler(os.Stdout, &slog.HandlerOptions{Level: level}))
}

func fatal(err error) {
	fmt.Fprintf(os.Stderr, "错误: %v\n", err)
	os.Exit(1)
}
