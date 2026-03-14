{
  description = "Python project using uv";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        python = pkgs.python312;

      in {
        devShells.default = pkgs.mkShell {
          packages = [
            python
            pkgs.uv
     pkgs.qt6.qtbase
        pkgs.python312Packages.pyqt6
          ];

          buildInputs = [
            pkgs.openssl
            pkgs.libffi
            pkgs.zlib
          ];

          shellHook = ''
            if [ ! -d .venv ]; then
              uv venv
            fi
            source .venv/bin/activate
          '';
        };
      });
}