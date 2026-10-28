{
  description = "Gesture IMU: STM32G031 firmware, recognition core, Rust host tools";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    # ST vendor sources (CMSIS + LL), pinned by flake.lock, never vendored into the repo.
    cmsis-core = {
      url = "github:STMicroelectronics/cmsis_core";
      flake = false;
    };
    cmsis-device-g0 = {
      url = "github:STMicroelectronics/cmsis_device_g0";
      flake = false;
    };
    stm32g0xx-hal-driver = {
      url = "github:STMicroelectronics/stm32g0xx_hal_driver";
      flake = false;
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      cmsis-core,
      cmsis-device-g0,
      stm32g0xx-hal-driver,
    }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in
    {
      formatter = forAllSystems (pkgs: pkgs.nixfmt);

      devShells = forAllSystems (
        pkgs:
        let
          # Runtime libraries eframe/winit load via dlopen.
          guiLibs = with pkgs; [
            libGL
            libxkbcommon
            wayland
            libx11
            libxcursor
            libxi
            libxrandr
          ];
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              # Build
              cmake
              ninja
              gcc-arm-embedded
              clang-tools

              # Flash / debug / serial
              openocd
              stlink
              tio

              # Rust
              cargo
              rustc
              clippy
              rustfmt
              rust-analyzer
              pkg-config
              udev
            ];

            buildInputs = guiLibs;

            CMSIS_CORE_DIR = "${cmsis-core}";
            CMSIS_DEVICE_G0_DIR = "${cmsis-device-g0}";
            STM32G0_LL_DIR = "${stm32g0xx-hal-driver}";

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath guiLibs;
          };
        }
      );
    };
}
