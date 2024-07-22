{
  description = "tetris-rs";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
      in
      {
        devShells.default = with pkgs; mkShell rec {
          LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
          buildInputs = [
            cargo-cache
            openssl
            pkg-config
            rust-bin.beta.latest.default

            # dev tools
            rust-analyzer

            # bevy deps
            udev
            alsa-lib
            vulkan-loader
            xorg.libX11
            xorg.libXcursor
            xorg.libXi
            xorg.libXrandr # To use the x11 feature
            libxkbcommon
            wayland # To use the wayland feature

            # for dynamic linking
            clang
            lld

            # for bevy_mod_debugdump graph visualization
            graphviz

          ];

          # drop into fish shell for development
          shellHook = ''
          '';
        };
      }
    );
}
