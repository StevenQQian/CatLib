# Installation Guide for CatLib

## Prerequisites

Before installing CatLib, ensure you have the following prerequisites:

- Visual Studio Code
- [PROS Extention](https://marketplace.visualstudio.com/items?itemName=sigbots.pros) for VSC
- A PROS 4 project

## Installation

Follow these steps to install CatLib:

### **Navigate to project:**
Open your project in Visual Studio Code. Click on the PROS icon in the left sidebar and then `Integrated Terminal`. Click on the newly opened terminal and type the following commands.

### **Install depots:**
This step adds a remote respoitory to your project so it can find the latest CatLib release link. This will make updating CatLib easier in the future. The second depot is for [Eigen](https://github.com/LemLib/Eigen), a CatLib dependency for handling units.

```bash
pros c add-depot catLib https://raw.githubusercontent.com/StevenQQian/CatLib/refs/heads/depot/stable.json
pros c add-depot Eigen https://raw.githubusercontent.com/StevenQQian/CatLib/refs/heads/depot/eigen.json
```

### **Install templates:**
```bash
pros c apply catLib
pros c apply Eigen
```

### **Include headers:**
Add the following line to `include/main.h` in your project:
```
#include "catLib/api.hpp"
```

## Uninstallation

To uninstall CatLib, run the following:
```bash
pros c uninstall catLib
pros c uninstall Eigen
```