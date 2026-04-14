#!/bin/bash
# fix_energest.sh
# Patches /home/user/contiki/core/net/rpl/rpl-dag.c to remove the
# "#include "dev/energest.h"" line that causes compilation failure.
# energest functions are globally available when ENERGEST_CONF_ON=1 —
# no explicit header include is needed (same as Version2's rpl-dag.c).

CONTIKI=${CONTIKI:-/home/user/contiki}
TARGET="$CONTIKI/core/net/rpl/rpl-dag.c"

if [ ! -f "$TARGET" ]; then
  echo "ERROR: $TARGET not found. Set CONTIKI env variable if different path."
  exit 1
fi

# Check if the bad include is present
if grep -q '#include "dev/energest.h"' "$TARGET"; then
  echo "Found bad include in $TARGET — patching..."

  # Remove the 3-line block:  #if ENERGEST_CONF_ON / #include "dev/energest.h" / #endif
  sed -i '/#if ENERGEST_CONF_ON/{
    N
    s/#if ENERGEST_CONF_ON\n#include "dev\/energest.h"/#if ENERGEST_CONF_ON\n\/\* energest.h not needed — functions are globally declared when ENERGEST_CONF_ON=1 *\//
  }' "$TARGET"

  # Also handle the case where it might be on separate lines differently
  # More robust: just remove the bad include line entirely
  sed -i '/#include "dev\/energest.h"/d' "$TARGET"

  echo "Patch applied. Verifying..."
  if grep -q 'energest.h' "$TARGET"; then
    echo "WARNING: energest.h still referenced in $TARGET — manual check needed."
  else
    echo "OK: energest.h include removed successfully."
    echo "The ENERGEST_CONF_ON guard remains. When the platform sets it to 1,"
    echo "energest_flush() and energest_type_time() are available without a header."
  fi

elif grep -q '#include "sys/energest.h"' "$TARGET"; then
  echo "Found 'sys/energest.h' include — removing (not needed)..."
  sed -i '/#include "sys\/energest.h"/d' "$TARGET"
  echo "Done."
else
  echo "No energest.h include found in $TARGET — no patch needed."
  echo "Checking for the include guard block..."
  grep -n 'energest' "$TARGET" | head -10
fi

echo ""
echo "Current energest-related lines in $TARGET:"
grep -n 'energest' "$TARGET"
