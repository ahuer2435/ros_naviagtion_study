FILE(REMOVE_RECURSE
  "CMakeFiles/move_base_gencfg"
  "devel/include/move_base/MoveBaseConfig.h"
  "devel/share/move_base/docs/MoveBaseConfig.dox"
  "devel/share/move_base/docs/MoveBaseConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/move_base/cfg/MoveBaseConfig.py"
  "devel/share/move_base/docs/MoveBaseConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/move_base_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
