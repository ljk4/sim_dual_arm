# Stable capability identifier for this scenario's fixed Teacher extension.
# A client must require exactly this identifier before it calls GetTeacherChunk.
# The old sim_api_msgs Teacher type used the bare name "teacher_chunk"; it is
# deliberately not accepted because the wire type (and therefore the package)
# changed even though the payload fields are unchanged.
string ID=panda_stack_api_msgs.teacher_chunk.v1
