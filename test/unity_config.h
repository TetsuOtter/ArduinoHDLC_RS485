#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

// Unity test framework configuration for Arduino

#define UNITY_INCLUDE_DOUBLE
#define UNITY_INCLUDE_FLOAT
#define UNITY_OUTPUT_CHAR(a) Serial.write(a)
#define UNITY_OUTPUT_FLUSH() Serial.flush()
#define UNITY_OUTPUT_START() Serial.begin(9600)
#define UNITY_OUTPUT_COMPLETE() Serial.flush()

#endif // UNITY_CONFIG_H
