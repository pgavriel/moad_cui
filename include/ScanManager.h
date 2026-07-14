#pragma once
/*
    ScanManager.h
    Data collection orchestration — single frame capture, folder preparation,
    turntable control, and full/custom/resume scan loops.
*/
#include <chrono>
#include "ThreadPool.h"

// Folder setup
void prepare_scan_output_folders();

// Core single-frame capture. Returns false on failure.
bool scan(ThreadPool* pool = nullptr);

// Turntable control
void rotate_turntable(int degree_inc);

// Post-scan metadata
void saveCameraConfig(std::string path);
void saveScanTime(std::chrono::milliseconds duration, std::string path);

// Scan orchestrators — called directly by menu or thin wrappers
bool fullScan();
bool customScan();
bool scanFromSaveState();
bool collectSampleData();