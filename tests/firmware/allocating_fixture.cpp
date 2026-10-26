// SPDX-License-Identifier: MIT
#include <vector>

// AR-1: intentionally reaches operator new for the symbols refusal control.
int main() {
    std::vector<int> values(8);
    const volatile int sink = values.front();
    return sink;
}
