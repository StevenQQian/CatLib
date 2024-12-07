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

### **Install depot:**
This step adds a remote respoitory to your project so it can find the latest CatLib release link. This will make updating CatLib easier in the future.

```bash
pros c add-depot catLib https://raw.githubusercontent.com/StevenQQian/CatLib/refs/heads/depot/stable.json
```

### **Install template:**
```bash
pros c apply catLib
```

## Verifying the Installation

To verify that CatLib has been installed correctly, run the following command:

```bash
pros terminal
```

You should see the output from the VEX V5 Brain indicating that CatLib is running.

## Troubleshooting

If you encounter any issues during installation, please refer to the [troubleshooting guide](./TROUBLESHOOTING.md) or open an issue on the [GitHub repository](https://github.com/yourusername/CatLib/issues).

## Uninstallation

To uninstall CatLib, simply delete the project directory.

For more detailed information, please refer to the [official documentation](https://github.com/yourusername/CatLib/wiki).
