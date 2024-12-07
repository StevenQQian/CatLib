# Installation Guide for CatLib

## Prerequisites

Before installing CatLib, ensure you have the following prerequisites:

- Visual Studio Code
- [PROS Extention](https://marketplace.visualstudio.com/items?itemName=sigbots.pros) for VSC
- A PROS 4 project

## Installation

Follow these steps to install CatLib:

1. **Install depot:**
	
	```bash
	git clone https://github.com/yourusername/CatLib.git
	```

2. **Navigate to the project directory:**

	```bash
	cd CatLib
	```

3. **Build the project:**

	```bash
	pros build
	```

4. **Upload the project to the VEX V5 Brain:**

	```bash
	pros upload
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
