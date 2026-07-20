#pragma once
#include <cstdint>

// -----------------------------------------------------------------------
//  קבועי גריד
// -----------------------------------------------------------------------
static constexpr float RESOLUTION = 0.2f;
static constexpr int   GRID_W     = 400;  // ← 80 מטר במקום 40
static constexpr int   GRID_H     = 400;

// -----------------------------------------------------------------------
//  ערכי עלות
// -----------------------------------------------------------------------
static constexpr uint8_t COST_FREE     =   0;
static constexpr uint8_t COST_INFLATED = 128;
static constexpr uint8_t COST_LETHAL   = 254;
static constexpr uint8_t COST_UNKNOWN  = 255;

// -----------------------------------------------------------------------
//  קבועי תצורה
// -----------------------------------------------------------------------
static constexpr float INFLATION_R             = 0.5f;   // was 0.2f — larger wall buffer (robot half-width + safety)
static constexpr int   OBJ_BODY_RADIUS_MIN     = 2;
static constexpr float DYNAMIC_SPEED_THRESHOLD = 0.05f;
static constexpr float DYN_SIGMA_D             = 10.0f;
static constexpr float DYN_SIGMA_TH            = 0.55f;
static constexpr int   DYN_REACH               = 30;
static constexpr float CONFIRM_BOOST           = 0.25f;
static constexpr float CURB_HEIGHT_THRESHOLD   = 0.05f;
