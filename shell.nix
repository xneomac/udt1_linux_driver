{ pkgs ? import (builtins.fetchTarball {
  url = "https://github.com/NixOS/nixpkgs/tarball/21.05";
  sha256 = "sha256:1ckzhh24mgz6jd1xhfgx0i9mijk6xjqxwsshnvq789xsavrmsc36";
}) { overlays = [ ]; } }:
with pkgs;
let
  here = toString ./.;
  pythonWithDeps = python36.override {
    packageOverrides = callPackage ./python-packages.nix { };
  };
in mkShell {
  buildInputs = with pythonWithDeps;
    [
      # pythonWithDeps.pkgs.pip
      # pythonWithDeps.pkgs.setuptools
      # cmake
      # gcc-arm-embedded
      # ninja
      # stlink
      # pythonWithDeps.pkgs.mbed-tools
      # pythonWithDeps.pkgs.mbed-cli
      # pythonWithDeps.pkgs.Jinja2
      # pythonWithDeps.pkgs.icetea
      # libusb
      gnumake
    ];
  PIP_PREFIX = "${here}/build/pip_packages";
  PYTHONPATH = "${here}/build/pip_packages/lib/python3.6/site-packages";
  SOURCE_DATE_EPOCH = "";
  shellHook = ''
    export PATH="$PATH:$(pwd)/build/pip_packages/bin"
  '';
}
