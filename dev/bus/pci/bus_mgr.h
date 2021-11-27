/*
 * Copyright (c) 2021 Travis Geiseblrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#include <sys/types.h>
#include <dev/bus/pci.h>

// C++ wrapper to convert lambdas and other function like things to the C api
template <typename T>
void pci_bus_mgr_visit_devices(T routine) {
    struct vdata {
        T &routine;
    };

    auto v = [](pci_location_t loc, void *cookie) {
        vdata *data = static_cast<vdata *>(cookie);
        data->routine(loc);
    };

    vdata data = { routine };
    pci_bus_mgr_visit_devices(v, &data);
}
