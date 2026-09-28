/*
 * main.c - Firmware SCARA RRP (Estacion 3, Grupo 2-B) - ESP32 + micro-ROS (Jazzy)
 *
 * Aqui vive la logica de la aplicacion (consignas, IK, control). Los modulos
 * de bajo nivel estan en components/:
 *   components/motors  -> PWM, direccion y finales de carrera
 *   components/homing  -> maquina de estados del homing
 *
 * Dos tareas:
 *   - control_task   (nucleo 1, cada 10 ms): homing y, mas adelante, PID + IK.
 *                    No depende de micro-ROS: si la USB se cae, sigue corriendo.
 *   - micro_ros_task (nucleo 0): comunicacion con ROS 2, con reconexion
 *                    automatica al agente (ya no hay que presionar EN).
 *
 * Interfaz ROS 2:
 *   /scara/cmd     (std_msgs/String, PC -> ESP32):
 *                  "start"  -> homing desde el motor 1
 *                  "resume" -> continua el homing donde quedo
 *                  "stop"   -> detiene todo
 *   /scara/status  (std_msgs/String, ESP32 -> PC): "IDLE", "HOMING M1 SEEK_A",
 *                  "ABORTED M2 SEEK_B", "HOMED". Se publica al cambiar y cada 1 s.
 *
 * Transporte: UART0 (GPIO1 TX / GPIO3 RX) a 115200. En menuconfig:
 *   micro-ROS Settings -> UART TXD = 1, RXD = 3; consola -> None.
 */
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "motors.h"
#include "homing.h"

/* ---------------- Parametros ---------------- */
#define CONTROL_PERIOD_MS     10
#define STATUS_PERIOD_MS      100     /* cada cuanto se revisa si publicar */
#define HEARTBEAT_MS          1000    /* publicar aunque no haya cambios */
#define AGENT_CHECK_MS        1000    /* cada cuanto verificar el agente */
#define CMD_BUF_LEN           32
#define STATUS_BUF_LEN        48

/* Si algo falla al crear entidades, devuelve false (no mata la tarea) */
#define RCRETURN(fn) { rcl_ret_t rc_ = (fn); if (rc_ != RCL_RET_OK) { return false; } }
#define RCSOFT(fn)   { rcl_ret_t rc_ = (fn); (void)rc_; }

/* ================================================================
 *  Tarea de control (nucleo 1)
 * ================================================================ */
static void control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        homing_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/* ================================================================
 *  micro-ROS (nucleo 0)
 * ================================================================ */
static rcl_allocator_t   allocator;
static rclc_support_t    support;
static rcl_node_t        node;
static rcl_subscription_t cmd_sub;
static rcl_publisher_t   status_pub;
static rcl_timer_t       status_timer;
static rclc_executor_t   executor;

static std_msgs__msg__String cmd_msg;
static std_msgs__msg__String status_msg;
static char cmd_buf[CMD_BUF_LEN];
static char status_buf[STATUS_BUF_LEN];

static uint32_t   last_seq = UINT32_MAX;
static TickType_t last_pub = 0;

/* El texto que llega NO trae '\0' al final: se compara usando .size */
static bool cmd_is(const std_msgs__msg__String *m, const char *word)
{
    size_t n = strlen(word);
    return m->data.size == n && strncmp(m->data.data, word, n) == 0;
}

static void cmd_callback(const void *msgin)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
    if (cmd_is(m, "start")) {
        homing_start();
    } else if (cmd_is(m, "resume")) {
        homing_resume();
    } else if (cmd_is(m, "stop")) {
        homing_abort();
    }
    /* comandos desconocidos se ignoran */
}

static void status_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    uint32_t s = homing_status_seq();
    TickType_t now = xTaskGetTickCount();
    if (s == last_seq && (now - last_pub) < pdMS_TO_TICKS(HEARTBEAT_MS)) {
        return;
    }
    homing_status_str(status_buf, sizeof(status_buf));
    status_msg.data.data = status_buf;
    status_msg.data.size = strlen(status_buf);
    status_msg.data.capacity = sizeof(status_buf);
    RCSOFT(rcl_publish(&status_pub, &status_msg, NULL));
    last_seq = s;
    last_pub = now;
}

static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();
    RCRETURN(rclc_support_init(&support, 0, NULL, &allocator));
    RCRETURN(rclc_node_init_default(&node, "scara_esp32", "", &support));

    RCRETURN(rclc_subscription_init_default(
        &cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "scara/cmd"));
    RCRETURN(rclc_publisher_init_default(
        &status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "scara/status"));
    RCRETURN(rclc_timer_init_default2(
        &status_timer, &support, RCL_MS_TO_NS(STATUS_PERIOD_MS), status_timer_cb, true));

    /* Memoria fija para el mensaje entrante (micro-ROS no hace malloc aqui) */
    cmd_msg.data.data = cmd_buf;
    cmd_msg.data.size = 0;
    cmd_msg.data.capacity = sizeof(cmd_buf);

    executor = rclc_executor_get_zero_initialized_executor();
    RCRETURN(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCRETURN(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                            &cmd_callback, ON_NEW_DATA));
    RCRETURN(rclc_executor_add_timer(&executor, &status_timer));

    last_seq = UINT32_MAX;   /* forzar publicacion inmediata al conectar */
    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFT(rcl_subscription_fini(&cmd_sub, &node));
    RCSOFT(rcl_publisher_fini(&status_pub, &node));
    RCSOFT(rcl_timer_fini(&status_timer));
    RCSOFT(rclc_executor_fini(&executor));
    RCSOFT(rcl_node_fini(&node));
    RCSOFT(rclc_support_fini(&support));
}

typedef enum { WAITING_AGENT, AGENT_CONNECTED } agent_state_t;

static void micro_ros_task(void *arg)
{
    agent_state_t st = WAITING_AGENT;
    TickType_t last_check = 0;

    for (;;) {
        switch (st) {
        case WAITING_AGENT:
            /* Intenta hablar con el agente; si responde, crea todo */
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (create_entities()) {
                    st = AGENT_CONNECTED;
                    last_check = xTaskGetTickCount();
                } else {
                    destroy_entities();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if ((xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(AGENT_CHECK_MS)) {
                last_check = xTaskGetTickCount();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    /* Agente perdido: limpiar y volver a esperar.
                     * El homing sigue corriendo en el nucleo 1. */
                    destroy_entities();
                    st = WAITING_AGENT;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
    }
}

/* ================================================================ */
static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    motors_init();

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif

    /* Control en el nucleo 1 con prioridad mayor que la comunicacion */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1);

    xTaskCreatePinnedToCore(micro_ros_task, "uros", CONFIG_MICRO_ROS_APP_STACK, NULL,
                            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
}
