#include"MQTT_driver.h"
#include"WiFi.h"
#include "task_app.h"
#include "sensor_driver.h"

void(* resetFunc) (void) = 0; //重启命令

/*WIFI初始化*/
void wifi_setup(){
    if(DEBUG_MODE){Serial.printf("Connecting to %s\n", wifi_ssid);}
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if(DEBUG_MODE){Serial.print(".");}
    }
    if(DEBUG_MODE){
        Serial.println("[debug]WIFI 已连接");
        Serial.print("[debug]WIFI IP地址: ");
        Serial.println(WiFi.localIP());}
    hass_debug_log("[SYS]WIFI连接成功！");
}

/*MQTT初始化*/
void mqtt_setup(){
    // 设置MQTT客户端
    mqttClient.setClient(tcpClient);//？
    mqttClient.setServer(mqtt_broker_addr, mqtt_broker_port); // 地址和端口
    mqttClient.setCallback(mqtt_callback);
    mqtt_client_id += String(WiFi.macAddress());// 每个客户端需要有唯一的ID，不然上线时会把其他相同ID的客户端踢下线
    if(DEBUG_MODE){Serial.println("[debug]MQTT客户端初始化");}
    //                            客户端ID              用户名         密码        WiFi遗嘱主题    清洁会话 QoS  遗嘱内容   
    if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password, mqtt_willTopic, false, 2, "offline"))
    {
        mqttClient.publish("esp32/state", "online"); // 连接成功后发布状态
        if(DEBUG_MODE)Serial.println("[debug]MQTT连接成功！");
        hass_debug_log("[sys]MQTT连接成功!");
        mqtt_subscribe_setup();
        if(DEBUG_MODE)Serial.println("[debug]MQTT订阅列表添加成功！");
        hass_debug_log("[sys]MQTT订阅列表添加成功！");
    }
}

/* 物联网Iot服务初始化函数 */
void setup_iot_server(){
    if(DEBUG_MODE) Serial.printf("\n[debug]WIFI Connecting to %s", wifi_ssid);
    wifi_setup();
    rssi = WiFi.RSSI(); // 获取网络连接质量
    mqtt_setup();
  }

/* MQTT回调函数 */
void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    /* char类型topic转String类型 */  
    int topic_length = strlen(topic); // 确定topic字符串的长度
    char topic_char[topic_length + 1]; // 创建一个足够长的字符数组来存储 topic，并添加一个空字符
    strncpy(topic_char, topic, topic_length); // 将 topic 复制到新的字符数组中，并确保以 null 结尾
    topic_char[topic_length] = '\0'; // 添加字符串结束符
    String str_topic = String(topic_char); // 将 char 数组转换为 String
    
    /* byte类型payload转String类型 */  
    char payload_char[length + 1];
    for (unsigned int i = 0; i < length; i++) {
        payload_char[i] = (char)payload[i];
    }
    payload_char[length] = '\0'; // 添加字符串结束符
    String str_payload = String(payload_char);

    if(str_topic != "" && str_payload != "") {
        enable_Iot_data_upload = false; //关闭数据上传 等待状态刷新
    }
        //例子
/*        if(str_topic == "esp32/differentiation/refrigeration"){
            if(str_payload == "ON") refrigeration_state_differentiation = true;
            else if(str_payload == "OFF") refrigeration_state_differentiation = false;
        }*/
          // 系统重启指令
    if(str_topic == "esp32/system/reboot"){
      if(str_payload == "reboot"){
        if(DEBUG_MODE)Serial.printf("\n\n[DEBUG]系统即将执行重启操作！\n\n");
        hass_debug_log("[SYS]即将重启！");
        if(DEBUG_MODE)Serial.printf("\n\n[DEBUG]系统重启！\n\n");
        resetFunc(); 
      }
    }

    if(DEBUG_MODE){
        Serial.printf("\n[DEBUG]收到MQTT消息 主题[%s], 长度[%d]\n", str_topic.c_str(), length);
        Serial.printf("[DEBUG]消息内容:%s",str_payload.c_str());
    }
        /*for (int i = 0; i < length; i++)
        {
            if(DEBUG_MODE)Serial.print((char)payload[i]);
        }*/
    if(DEBUG_MODE)Serial.println("\n[DEBUG]----------------END----------------");
    enable_Iot_data_upload = true; // 启用数据上传   
}


/* MQTT订阅列表初始化 */
void mqtt_subscribe_setup(){
    mqttClient.subscribe("esp32/system/reboot");  // 系统重启指令 
    mqttClient.subscribe("esp32/");  
}

/* 连接状态检查及重连函数 */
void connect_check(){
    if(WiFi.status() != WL_CONNECTED){  // 如果WIFI未连接
      wifi_setup();
    }
    rssi = WiFi.RSSI(); // 获取网络连接质量
    if (!mqttClient.connected()) // 如果MQTT未连接
      {
        mqtt_client_id += String(WiFi.macAddress()); // 每个客户端需要有唯一的ID，不然上线时会把其他相同ID的客户端踢下线
        if(DEBUG_MODE)Serial.println("[DEBUG]MQTT正在连接...");
        if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password, mqtt_willTopic, false, 2, "offline"))// 尝试连接MQTT服务器
        {
          mqttClient.publish("esp32/state", "online"); // 连接成功后发布状态
          if(DEBUG_MODE)Serial.println("[DEBUG]MQTT连接成功！");
          hass_debug_log("[SYS]MQTT连接成功!");
          mqtt_subscribe_setup();
          if(DEBUG_MODE)Serial.println("[DEBUG]MQTT订阅列表添加成功！");
          hass_debug_log("[SYS]MQTTMQTT订阅列表添加成功！");
        }
      }
}


/* HASS debug日志传输函数 */
void hass_debug_log(char *log){
    mqttClient.publish("esp32/system/debug", log); 
  }