#ifndef JSON_H
#define JSON_H

#include <string>
#define ARDUINOJSON_EMBEDDED_MODE 1
#include "../ArduinoJson/ArduinoJson-v5.13.0.h"

typedef int (*MethodHandler)(JsonObject& request);

struct HandlerData
{
  std::string Name;
  MethodHandler Callback;
};

void JsonRpcAddHandler(char* methodName, MethodHandler handler);
int JsonRpcProcess(char* json, bool async = false);
int JsonRpcResult(char* id, JsonObject& results, char* output, size_t size);
int JsonRpcError(char* id, long errorCode, char* errorMessage, char* output, size_t size);
int JsonRpcInvoke(char* id, char* method, JsonObject& params, char* output, size_t size);

#endif