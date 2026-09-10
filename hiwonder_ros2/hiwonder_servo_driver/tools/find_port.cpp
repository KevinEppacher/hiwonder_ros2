// Copyright 2026 Kevin Eppacher
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <thread>

namespace fs = std::filesystem;

std::set<std::string> findAvailablePorts()
{
  std::set<std::string> ports;

  const fs::path tty_root{"/sys/class/tty"};

  for (const auto & entry : fs::directory_iterator(tty_root)) {
    const std::string name = entry.path().filename().string();

    if (!name.starts_with("ttyACM") && !name.starts_with("ttyUSB")) {
      continue;
    }

    const fs::path device = entry.path() / "device";

    if (fs::exists(device)) {
      ports.insert("/dev/" + name);
    }
  }

  return ports;
}

int main()
{
  std::cout << "Finding all available ports for the Hiwonder servo bus.\n";

  const auto ports_before = findAvailablePorts();
  ports_before.empty() ? std::cout << "No ports found.\n" :
    std::cout << "Ports before disconnecting:\n";

  for (const auto & port : ports_before) {
    std::cout << "  " << port << '\n';
  }

  std::cout
        << "\nDisconnect the USB cable from the Hiwonder servo bus "
        << "and press Enter.\n";

  std::cin.get();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto ports_after = findAvailablePorts();

  std::set<std::string> difference;

  for (const auto & port : ports_before) {
    if (!ports_after.contains(port)) {
      difference.insert(port);
    }
  }

  if (difference.size() == 1) {
    std::cout
            << "The port of the Hiwonder servo bus is '"
            << *difference.begin()
            << "'\n";

    std::cout << "Reconnect the USB cable.\n";
    return 0;
  }

  if (difference.empty()) {
    std::cerr << "Could not detect the port. No difference was found.\n";
  } else {
    std::cerr
            << "Could not detect the port. More than one port disappeared.\n";
  }

  return 1;
}
