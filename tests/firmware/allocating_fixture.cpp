// SPDX-License-Identifier: MIT
#include <vector>

// Allocates through operator new so the symbols gate detects the violation
// (AR-1).
int main() {
    std::vector<int> values(8);
    const volatile int sink = values.front();
    return sink;
}
