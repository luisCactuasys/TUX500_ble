#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <string>
#include <list>

#include "json.h"
// extern "C" {
  #include "tools.h"
// }

extern int printf_d(const char* fmt, ...);


std::list<HandlerData*> HandlerDataList;

void JsonRpcAddHandler(char* methodName, MethodHandler callback)
{
  HandlerData* handler = new HandlerData();
  handler->Name = methodName;
  handler->Callback = callback;
  HandlerDataList.push_back(handler);
}

void* _JsonRpcProcess(void* arg)
{
  char* json = (char*)arg;
  StaticJsonBuffer<4000> jsonReqBuffer;
  JsonObject& root = jsonReqBuffer.parseObject(json);

  const char* methodName = root["method"].as<char*>();
  printf_d("\n[JsonRpcProcess] Got call for method '%s'", methodName);
  
  MethodHandler defaultHandler = NULL;
  int result = -99;
  for (std::list<HandlerData*>::const_iterator iterator = HandlerDataList.begin(), end = HandlerDataList.end(); iterator != end; ++iterator)
  {
    if (strcasecmp((*iterator)->Name.c_str(), methodName) == 0)
    {
      // It's this method
        result = (*iterator)->Callback(root);
        defaultHandler = NULL;
        break;
    }

    if ((*iterator)->Name == "*")
    {
      defaultHandler = (*iterator)->Callback;
    }
  }

  // Process no such method response
  if (defaultHandler)
    result = defaultHandler(root);

  free(json);
  return (void*)(intptr_t)result;
}

int JsonRpcProcess(char* json, bool async)
{
  char* json2 = strdup(json);
  {
    StaticJsonBuffer<4000> jsonReqBuffer;
    JsonObject& root = jsonReqBuffer.parseObject(json);

    if (!root.success())
    {
      printf_d("\n[JsonRpcProcess] parseObject() failed");
      free(json2);
      return -1;
    }

    if (root["jsonrpc"] != "2.0")
    {
      printf_d("\n[JsonRpcProcess] Not JSON-RPC 2.0");
      free(json2);
      return -2;
    }

    if (!root["method"].is<char*>())
    {
      printf_d("\n[JsonRpcProcess] No valid 'method' argument");
      free(json2);
      return -3;
    }
  }
  
  if (async)
  {
    void* handle;
    framework_CreateThread(&handle, _JsonRpcProcess, json2);
    return 0;
  }
  else
    return (int)(int64_t)_JsonRpcProcess(json2);
}

int JsonRpcResult(char* id, JsonObject& result, char* output, size_t size)
{
  StaticJsonBuffer<4000> jsonRespBuffer;
  JsonObject& jsonResponse = jsonRespBuffer.createObject();
  
  jsonResponse["jsonrpc"] = "2.0";
  if (id != NULL)
    jsonResponse["id"] = id;
  jsonResponse["result"] = result;
    
  jsonResponse.printTo(output, size);
  return 0;
}

int JsonRpcError(char* id, long errorCode, char* errorMessage, char* output, size_t size)
{
  StaticJsonBuffer<4000> jsonRespBuffer;
  JsonObject& jsonResponse = jsonRespBuffer.createObject();
  
  jsonResponse["jsonrpc"] = "2.0";
  if (id != NULL)
    jsonResponse["id"] = id;

  StaticJsonBuffer<4000> jsonRespError;
  JsonObject& jsonError = jsonRespError.createObject();

  jsonError["code"] = errorCode;
  if (errorMessage)
    jsonError["message"] = errorMessage;
  jsonResponse["error"] = jsonError;
    
  jsonResponse.printTo(output, size);
  return 0;
}

int JsonRpcInvoke(char* id, char* method, JsonObject& params, char* output, size_t size)
{
  StaticJsonBuffer<4000> jsonReqBuffer;
  JsonObject& jsonRequest = jsonReqBuffer.createObject();
  
  jsonRequest["jsonrpc"] = "2.0";
  if (id != NULL)
  {
    if (id[0] == '"')
      jsonRequest["id"] = &id[1];
    else
      jsonRequest["id"] = atol(id);
  }
  jsonRequest["method"] = method;
  if (params.size())
    jsonRequest["params"] = params;

  jsonRequest.printTo(output, size);
  return 0;
}