#altitude control command info

time time_stamp
float64 target_altitude #Altitude to try to achieve in mm
float64 current_p_term #(targetAltitude - fabs(altitude))
float64 current_i_term  #targetAltitudeITerm = targetAltitudeITerm + pTerm

