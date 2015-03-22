FILE(REMOVE_RECURSE
  "CMakeFiles/ardrone_command_generate_messages_py"
  "devel/lib/python2.7/dist-packages/ardrone_command/msg/_serialized_ardrone_command.py"
  "devel/lib/python2.7/dist-packages/ardrone_command/msg/_serialized_ardrone_command_part.py"
  "devel/lib/python2.7/dist-packages/ardrone_command/srv/_commandInterface.py"
  "devel/lib/python2.7/dist-packages/ardrone_command/msg/__init__.py"
  "devel/lib/python2.7/dist-packages/ardrone_command/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ardrone_command_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
