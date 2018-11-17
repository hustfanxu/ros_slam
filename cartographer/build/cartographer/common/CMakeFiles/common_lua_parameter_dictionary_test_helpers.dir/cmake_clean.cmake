file(REMOVE_RECURSE
  "libcommon_lua_parameter_dictionary_test_helpers.pdb"
  "libcommon_lua_parameter_dictionary_test_helpers.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_lua_parameter_dictionary_test_helpers.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
