{pkgs ? import <nixpkgs> {}}:
pkgs.stdenv.mkDerivation {
  name = "nanocut-firmware-dev";
  nativeBuildInputs = with pkgs; [
    platformio
  ];
  buildInputs = with pkgs; [
  ];
}
