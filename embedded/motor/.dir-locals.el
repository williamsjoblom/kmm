((c-mode
  (flycheck-gcc-args . ("-Os" "-mmcu=atmega328p" "-std=gnu99"))
  (flycheck-c/c++-gcc-executable . "/usr/bin/avr-gcc")
  (flycheck-disabled-checkers . (c/c++-clang irony))))
