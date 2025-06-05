/*
 * Copyright (c) 2025 Croxel Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MAX9867_NODE DT_NODELABEL(max9867)

#if DT_ON_BUS(MAX9867_NODE, i2c)
bool init_max9867_i2c(void);
#endif
