; Auto-generated. Do not edit!


(cl:in-package loop_action-msg)


;//! \htmlinclude KeyFrameHandleResult.msg.html

(cl:defclass <KeyFrameHandleResult> (roslisp-msg-protocol:ros-message)
  ((keyframe_num
    :reader keyframe_num
    :initarg :keyframe_num
    :type cl:integer
    :initform 0))
)

(cl:defclass KeyFrameHandleResult (<KeyFrameHandleResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyFrameHandleResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyFrameHandleResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name loop_action-msg:<KeyFrameHandleResult> is deprecated: use loop_action-msg:KeyFrameHandleResult instead.")))

(cl:ensure-generic-function 'keyframe_num-val :lambda-list '(m))
(cl:defmethod keyframe_num-val ((m <KeyFrameHandleResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader loop_action-msg:keyframe_num-val is deprecated.  Use loop_action-msg:keyframe_num instead.")
  (keyframe_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyFrameHandleResult>) ostream)
  "Serializes a message object of type '<KeyFrameHandleResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'keyframe_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'keyframe_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'keyframe_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'keyframe_num)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyFrameHandleResult>) istream)
  "Deserializes a message object of type '<KeyFrameHandleResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'keyframe_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'keyframe_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'keyframe_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'keyframe_num)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyFrameHandleResult>)))
  "Returns string type for a message object of type '<KeyFrameHandleResult>"
  "loop_action/KeyFrameHandleResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyFrameHandleResult)))
  "Returns string type for a message object of type 'KeyFrameHandleResult"
  "loop_action/KeyFrameHandleResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyFrameHandleResult>)))
  "Returns md5sum for a message object of type '<KeyFrameHandleResult>"
  "89b35da2e44418098ba2504142cdc556")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyFrameHandleResult)))
  "Returns md5sum for a message object of type 'KeyFrameHandleResult"
  "89b35da2e44418098ba2504142cdc556")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyFrameHandleResult>)))
  "Returns full string definition for message of type '<KeyFrameHandleResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 keyframe_num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyFrameHandleResult)))
  "Returns full string definition for message of type 'KeyFrameHandleResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 keyframe_num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyFrameHandleResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyFrameHandleResult>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyFrameHandleResult
    (cl:cons ':keyframe_num (keyframe_num msg))
))
