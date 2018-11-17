file(REMOVE_RECURSE
  "libcommon_port.pdb"
  "libcommon_port.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_port.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
