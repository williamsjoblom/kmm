((c-mode
  (flycheck-gcc-args . ("-0s" "-mmcu=atmega328" "-std=gnu99"))
  (flycheck-c/c++-gcc-executable . "/usr/bin/avr-gcc")
  (flycheck-disabled-checkers . (c/c++-clang irony))))
