/* Simple HTTP server for ESP32.
 * Copyright Ivan Grokhotkov, 2017.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "my_http_server.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/lock.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "rom/queue.h"

#include "esp_log.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include "http_parser.h"

#define HTTP_PARSE_BUF_MAX_LEN 256

typedef enum {
    HTTP_PARSING_URI,                //!< HTTP_PARSING_URI
    HTTP_PARSING_HEADER_NAME,        //!< HTTP_PARSING_HEADER_NAME
    HTTP_PARSING_HEADER_VALUE,       //!< HTTP_PARSING_HEADER_VALUE
    HTTP_PARSING_REQUEST_BODY,       //!< HTTP_PARSING_REQUEST_BODY
    HTTP_REQUEST_DONE,               //!< HTTP_REQUEST_DONE
    HTTP_COLLECTING_RESPONSE_HEADERS,//!< HTTP_COLLECTING_RESPONSE_HEADERS
    HTTP_SENDING_RESPONSE_BODY,      //!< HTTP_SENDING_RESPONSE_BODY
    HTTP_DONE,                       //!< HTTP_DONE
} http_state_t;

typedef struct http_header_t {
    char* name;
    char* value;
    SLIST_ENTRY(http_header_t) list_entry;
} http_header_t;

typedef SLIST_HEAD(http_header_list_t, http_header_t) http_header_list_t;

typedef struct {
    http_handler_fn_t cb;
    void* ctx;
} http_form_handler_t;

typedef struct http_handler_t{
    char* uri_pattern;
    int method;
    int events;
    http_handler_fn_t cb;
    void* ctx;
    SLIST_ENTRY(http_handler_t) list_entry;
} http_handler_t;


struct http_context_ {
    http_server_t server;
    http_state_t state;
    int event;
    char* uri;
    char parse_buffer[HTTP_PARSE_BUF_MAX_LEN];
    char* request_header_tmp;
    struct netconn *conn;
    http_parser parser;
    http_header_list_t request_headers;
    int response_code;
    http_header_list_t response_headers;
    size_t expected_response_size;
    size_t accumulated_response_size;
    http_handler_t* handler;
    const char* data_ptr;
    size_t data_size;
    http_header_list_t request_args;
};


struct http_server_context_ {
    int port;
    err_t server_task_err;
    struct netconn* server_conn;
    TaskHandle_t task;
    EventGroupHandle_t start_done;
    SLIST_HEAD(, http_handler_t) handlers;
    _lock_t handlers_lock;
    struct http_context_ connection_context;
};

#define SERVER_STARTED_BIT BIT(0)
#define SERVER_DONE_BIT BIT(1)


static const char* http_response_code_to_str(int code);
static esp_err_t add_keyval_pair(http_header_list_t *list, const char* name, const char* val);

static const char* TAG = "http_server";

esp_err_t http_register_handler(http_server_t server,
        const char* uri_pattern, int method,
        int events, http_handler_fn_t callback, void* callback_arg)
{
    http_handler_t* new_handler = (http_handler_t*) calloc(1, sizeof(*new_handler));
    if (new_handler == NULL) {
        return ESP_ERR_NO_MEM;
    }

    new_handler->uri_pattern = strdup(uri_pattern);
    new_handler->cb = callback;
    new_handler->ctx = callback_arg;
    new_handler->method = method;
    new_handler->events = events;

    _lock_acquire(&server->handlers_lock);
    /* FIXME: Handlers will be checked in the reverse order */
    SLIST_INSERT_HEAD(&server->handlers, new_handler, list_entry);
    _lock_release(&server->handlers_lock);
    return ESP_OK;
}

static http_handler_t* http_find_handler(http_server_t server, const char* uri, int method)
{
    http_handler_t* it;
    _lock_acquire(&server->handlers_lock);
    SLIST_FOREACH(it, &server->handlers, list_entry) {
        if (strcasecmp(uri, it->uri_pattern) == 0
            && method == it->method) {
            break;
        }
    }
    _lock_release(&server->handlers_lock);
    return it;
}

static int append_parse_buffer(http_context_t ctx, const char* at, size_t length)
{
    if (length > HTTP_PARSE_BUF_MAX_LEN - strlen(ctx->parse_buffer) - 1) {
        ESP_LOGW(TAG, "%s: len=%d > %d", __func__, length, HTTP_PARSE_BUF_MAX_LEN - strlen(ctx->parse_buffer) - 1);
        return 1;
    }
    strncat(ctx->parse_buffer, at, length);
    ESP_LOGV(TAG, "%s: len=%d, '%s'", __func__, length, ctx->parse_buffer);
    return 0;
}

static void clear_parse_buffer(http_context_t ctx)
{
#ifdef NDEBUG
    ctx->parse_buffer[0] = 0;
#else
    memset(ctx->parse_buffer, 0, sizeof(ctx->parse_buffer));
#endif
}

static void header_name_done(http_context_t ctx)
{
    ctx->request_header_tmp = strdup(ctx->parse_buffer);
    clear_parse_buffer(ctx);
}

static void header_value_done(http_context_t ctx)
{
    const char* value = ctx->parse_buffer;
    const char* name = ctx->request_header_tmp;
    ESP_LOGD(TAG, "Got header: '%s': '%s'", name, value);
    add_keyval_pair(&ctx->request_headers, name, value);
    free(ctx->request_header_tmp);
    ctx->request_header_tmp = NULL;
    clear_parse_buffer(ctx);
}

static int http_url_cb(http_parser* parser, const char *at, size_t length)
{
    http_context_t ctx = (http_context_t) parser->data;
    return append_parse_buffer(ctx, at, length);
}

static bool invoke_handler(http_context_t ctx, int event)
{
    if (ctx->handler && (ctx->handler->events & event) != 0) {
        ctx->event = event;
        (*ctx->handler->cb)(ctx, ctx->handler->ctx);
        ctx->event = 0;
        return true;
    }
    return false;
}


static int http_headers_done_cb(http_parser* parser)
{
    http_context_t ctx = (http_context_t) parser->data;
    if (ctx->state == HTTP_PARSING_HEADER_VALUE) {
        header_value_done(ctx);
    }
    invoke_handler(ctx, HTTP_HANDLE_HEADERS);
    ctx->state = HTTP_PARSING_REQUEST_BODY;
    return 0;
}

static int parse_hex_digit(char hex)
{
    switch (hex) {
        case '0' ... '9': return hex - '0';
        case 'a' ... 'f': return hex - 'a' + 0xa;
        case 'A' ... 'F': return hex - 'A' + 0xA;
        default:
            return -1;
    }
}

static char* urldecode(const char* str, size_t len)
{
    ESP_LOGV(TAG, "urldecode: '%.*s'", len, str);

    const char* end = str + len;
    char* out = calloc(1, len + 1);
    char* p_out = out;
    while (str != end) {
        char c = *str++;
        if (c != '%') {
            *p_out = c;
        } else {
            if (str + 2 > end) {
                /* Unexpected end of string */
                return NULL;
            }
            int high = parse_hex_digit(*str++);
            int low = parse_hex_digit(*str++);
            if (high == -1 || low == -1) {
                /* Unexpected character */
                return NULL;
            }
            *p_out = high * 16 + low;
        }
        ++p_out;
    }
    *p_out = 0;
    ESP_LOGV(TAG, "urldecode result: '%s'", out);
    return out;
}

static void parse_urlencoded_args(http_context_t ctx, const char* str, size_t len)
{
    const char* end = str + len;
    const int READING_KEY = 1;
    const int READING_VAL = 2;
    int state = READING_KEY;
    const char* token_start = str;
    char* key = NULL;
    char* value = NULL;
    for (const char* pos = str; pos < end; ++pos) {
        char c = *pos;
        if (c == '=' && state == READING_KEY) {
            key = urldecode(token_start, pos - token_start);
            state = READING_VAL;
            token_start = pos + 1;
        } else if (c == '&' && state == READING_VAL) {
            value = urldecode(token_start, pos - token_start);
            state = READING_KEY;
            token_start = pos + 1;
            ESP_LOGD(TAG, "Got request argument, '%s': '%s'", key, value);
            add_keyval_pair(&ctx->request_args, key, value);
            free(key);
            key = NULL;
            free(value);
            value = NULL;
        }
    }
    if (state == READING_VAL) {
        value = urldecode(token_start, end - token_start);
        ESP_LOGD(TAG, "Got request argument, '%s': '%s'", key, value);
        add_keyval_pair(&ctx->request_args, key, value);
        free(key);
        key = NULL;
        free(value);
        value = NULL;
    }
}

static void uri_done(http_context_t ctx)
{
    /* Check for query argument string */
    char* query_str = strchr(ctx->parse_buffer, '?');
    if (query_str != NULL) {
        *query_str = 0;
        ++query_str;
    }
    ctx->uri = strdup(ctx->parse_buffer);
    ESP_LOGD(TAG, "Got URI: '%s'", ctx->uri);
    if (query_str) {
        parse_urlencoded_args(ctx, query_str, strlen(query_str));
    }

    ctx->handler = http_find_handler(ctx->server, ctx->uri, (int) ctx->parser.method);
    invoke_handler(ctx, HTTP_HANDLE_URI);
    clear_parse_buffer(ctx);
}


static int http_header_name_cb(http_parser* parser, const char *at, size_t length)
{
    ESP_LOGV(TAG, "%s", __func__);
    http_context_t ctx = (http_context_t) parser->data;
    if (ctx->state == HTTP_PARSING_URI) {
        uri_done(ctx);
        ctx->state = HTTP_PARSING_HEADER_NAME;
    } else if (ctx->state == HTTP_PARSING_HEADER_VALUE) {
        header_value_done(ctx);
        ctx->state = HTTP_PARSING_HEADER_NAME;
    }
    return append_parse_buffer(ctx, at, length);
}

static int http_header_value_cb(http_parser* parser, const char *at, size_t length)
{
    ESP_LOGV(TAG, "%s", __func__);
    http_context_t ctx = (http_context_t) parser->data;
    if (ctx->state == HTTP_PARSING_HEADER_NAME) {
        header_name_done(ctx);
        ctx->state = HTTP_PARSING_HEADER_VALUE;
    }
    return append_parse_buffer(ctx, at, length);
}

static int http_body_cb(http_parser* parser, const char *at, size_t length)
{
    ESP_LOGV(TAG, "%s", __func__);
    http_context_t ctx = (http_context_t) parser->data;
    ctx->data_ptr = at;
    ctx->data_size = length;
    invoke_handler(ctx, HTTP_HANDLE_DATA);
    ctx->data_ptr = NULL;
    ctx->data_size = 0;
    return 0;
}

static int http_message_done_cb(http_parser* parser)
{
    ESP_LOGV(TAG, "%s", __func__);
    http_context_t ctx = (http_context_t) parser->data;
    ctx->state = HTTP_REQUEST_DONE;
    return 0;
}

const char* http_request_get_header(http_context_t ctx, const char* name)
{
    http_header_t* it;
    SLIST_FOREACH(it, &ctx->request_headers, list_entry) {
        if (strcasecmp(name, it->name) == 0) {
            return it->value;
        }
    }
    return NULL;
}

int http_request_get_event(http_context_t ctx)
{
    return ctx->event;
}

const char* http_request_get_uri(http_context_t ctx)
{
    return ctx->uri;
}

int http_request_get_method(http_context_t ctx)
{
    return (int) ctx->parser.method;
}

esp_err_t http_request_get_data(http_context_t ctx, const char** out_data_ptr, size_t* out_size)
{
    if (ctx->event != HTTP_HANDLE_DATA) {
        return ESP_ERR_INVALID_STATE;
    }
    *out_data_ptr = ctx->data_ptr;
    *out_size = ctx->data_size;
    return ESP_OK;
}

static void form_data_handler_cb(http_context_t http_ctx, void* ctx)
{
    http_form_handler_t* form_ctx = (http_form_handler_t*) ctx;
    int event = http_request_get_event(http_ctx);
    if (event != HTTP_HANDLE_DATA) {
        (*form_ctx->cb)(http_ctx, form_ctx->ctx);
    } else {
        const char* str;
        size_t len;
        http_request_get_data(http_ctx, &str, &len);
        parse_urlencoded_args(ctx, str, len);
    }
}

esp_err_t http_register_form_handler(http_server_t server, const char* uri_pattern, int method,
                                    int events, http_handler_fn_t callback, void* callback_arg)
{
    http_form_handler_t* inner_handler = calloc(1, sizeof(*inner_handler));
    if (inner_handler == NULL) {
        return ESP_ERR_NO_MEM;
    }

    inner_handler->cb = callback;
    inner_handler->ctx = callback_arg;

    esp_err_t res = http_register_handler(server, uri_pattern, method,
            events | HTTP_HANDLE_DATA, &form_data_handler_cb, inner_handler);
    if (res != ESP_OK) {
        free(inner_handler);
    }
    return res;
}


static esp_err_t lwip_err_to_esp_err(err_t e)
{
    switch (e) {
        case ERR_OK: return ESP_OK;
        case ERR_MEM: return ESP_ERR_NO_MEM;
        case ERR_TIMEOUT: return ESP_ERR_TIMEOUT;
        default:
            return ESP_FAIL;
    }
}

static void headers_list_clear(http_header_list_t* list)
{
    http_header_t  *it, *next;
    SLIST_FOREACH_SAFE(it, list, list_entry, next) {
        SLIST_REMOVE(list, it, http_header_t, list_entry);
        free(it); /* frees memory allocated for header, name, and value */
    }
}

static esp_err_t http_add_content_length_header(http_context_t http_ctx, size_t value)
{
    char size_str[11];
    itoa(value, size_str, 10);
    return http_response_set_header(http_ctx, "Content-length", size_str);
}

static esp_err_t http_send_response_headers(http_context_t http_ctx)
{
    assert(http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS);

    /* Calculate total size of all the headers, allocate a buffer */
    size_t total_headers_size = 0;

    /* response_code may be == 0, if we are sending headers for multipart
     * response part. In this case, don't send the response code line.
     */
    if (http_ctx->response_code > 0) {
        total_headers_size += 16 /* HTTP/1.1, code, CRLF */
                + strlen(http_response_code_to_str(http_ctx->response_code));
    }
    http_header_t* it;
    SLIST_FOREACH(it, &http_ctx->response_headers, list_entry) {
        total_headers_size += strlen(it->name) + strlen(it->value) + 4 /* ": ", CRLF */;
    }
    total_headers_size += 3; /* Final CRLF, '\0' terminator */
    char* headers_buf = calloc(1, total_headers_size);
    if (headers_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* Write response */
    size_t buf_size = total_headers_size;
    char* buf_ptr = headers_buf;
    int len;
    if (http_ctx->response_code > 0) {
        len = snprintf(buf_ptr, buf_size, "HTTP/1.1 %d %s\r\n",
                http_ctx->response_code, http_response_code_to_str(http_ctx->response_code));
        assert(len < buf_size);
        buf_size -= len;
        buf_ptr += len;
    }

    /* Write response headers */
    SLIST_FOREACH(it, &http_ctx->response_headers, list_entry) {
        len = snprintf(buf_ptr, buf_size, "%s: %s\r\n", it->name, it->value);
        assert(len < buf_size);
        buf_size -= len;
        buf_ptr += len;
    }

    /* Final CRLF */
    len = snprintf(buf_ptr, buf_size, "\r\n");
    assert(len < buf_size);
    buf_size -= len;
    buf_ptr += len;

    headers_list_clear(&http_ctx->response_headers);

    err_t err = netconn_write(http_ctx->conn, headers_buf, strlen(headers_buf), NETCONN_COPY);
    free(headers_buf);

    http_ctx->state = HTTP_SENDING_RESPONSE_BODY;

    return lwip_err_to_esp_err(err);
}

/* Common function called by http_response_begin and http_response_begin_multipart */
static esp_err_t http_response_begin_common(http_context_t http_ctx, const char* content_type, size_t response_size)
{
    esp_err_t err = http_response_set_header(http_ctx, "Content-type", content_type);
    if (err != ESP_OK) {
        return err;
    }
    http_ctx->expected_response_size = response_size;
    http_ctx->accumulated_response_size = 0;
    if (response_size != HTTP_RESPONSE_SIZE_UNKNOWN) {
        err = http_add_content_length_header(http_ctx, response_size);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t http_response_begin(http_context_t http_ctx, int code, const char* content_type, size_t response_size)
{
    if (http_ctx->state != HTTP_COLLECTING_RESPONSE_HEADERS) {
        return ESP_ERR_INVALID_STATE;
    }
    http_ctx->response_code = code;
    return http_response_begin_common(http_ctx, content_type, response_size);
}

esp_err_t http_response_write(http_context_t http_ctx, const http_buffer_t* buffer)
{
    if (http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS) {
        esp_err_t err = http_send_response_headers(http_ctx);
        if (err != ESP_OK) {
            return err;
        }
    }
    const int flag = buffer->data_is_persistent ? NETCONN_NOCOPY : NETCONN_COPY;
    size_t len = buffer->size ? buffer->size : strlen((const char*) buffer->data);
    err_t rc = netconn_write(http_ctx->conn, buffer->data, len, flag);
    if (rc != ESP_OK) {
        ESP_LOGD(TAG, "netconn_write rc=%d", rc);
    } else {
        http_ctx->accumulated_response_size += len;
    }
    return lwip_err_to_esp_err(rc);
}


esp_err_t http_response_end(http_context_t http_ctx)
{
    size_t expected = http_ctx->expected_response_size;
    size_t actual = http_ctx->accumulated_response_size;
    if (expected != HTTP_RESPONSE_SIZE_UNKNOWN && expected != actual) {
        ESP_LOGW(TAG, "Expected response size: %d, actual: %d", expected, actual);
    }
    http_ctx->state = HTTP_DONE;
    return ESP_OK;
}

esp_err_t http_response_begin_multipart(http_context_t http_ctx, const char* content_type, size_t response_size)
{
    if (http_ctx->state == HTTP_COLLECTING_RESPONSE_HEADERS) {
        http_send_response_headers(http_ctx);
        http_ctx->response_code = 0;
    }
    http_ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
    return http_response_begin_common(http_ctx, content_type, response_size);
}

esp_err_t http_response_end_multipart(http_context_t http_ctx, const char* boundary)
{
    size_t expected = http_ctx->expected_response_size;
    size_t actual = http_ctx->accumulated_response_size;
    if (expected != HTTP_RESPONSE_SIZE_UNKNOWN && expected != actual) {
        ESP_LOGW(TAG, "Expected response size: %d, actual: %d", expected, actual);
    }
    /* reset expected_response_size so that http_response_end doesn't complain */
    http_ctx->expected_response_size = HTTP_RESPONSE_SIZE_UNKNOWN;

    const http_buffer_t buf = { .data = boundary };
    esp_err_t ret = http_response_write(http_ctx, &buf);
    http_ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
    return ret;
}

static esp_err_t add_keyval_pair(http_header_list_t *list, const char* name, const char* val)
{
    size_t name_len = strlen(name) + 1;
    size_t val_len = strlen(val) + 1;
    /* Allocate memory for the structure, name, and value, in one go */
    size_t buf_len = sizeof(http_header_t) + name_len + val_len;
    char* buf = (char*) calloc(1, buf_len);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    http_header_t* new_header = (http_header_t*) buf;
    new_header->name = buf + sizeof(http_header_t);
    new_header->value = new_header->name + name_len;
    strcpy(new_header->name, name);
    strcpy(new_header->value, val);
    SLIST_INSERT_HEAD(list, new_header, list_entry);
    return ESP_OK;
}

esp_err_t http_response_set_header(http_context_t http_ctx, const char* name, const char* val)
{
    return add_keyval_pair(&http_ctx->response_headers, name, val);
}


static void http_send_not_found_response(http_context_t http_ctx)
{
    http_response_begin(http_ctx, 404, "text/plain", HTTP_RESPONSE_SIZE_UNKNOWN);
    const http_buffer_t buf = {
            .data = "Not found",
            .data_is_persistent = true
    };
    http_response_write(http_ctx, &buf);
    http_response_end(http_ctx);
}


static const char* http_response_code_to_str(int code)
{
    switch (code) {
        case 200: return "OK";
        case 204: return "No Content";
        case 301: return "Moved Permanently";
        case 302: return "Found";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 500: return "Internal Server Error";
        default:  return "";
      }
}


static void http_handle_connection(http_server_t server, struct netconn *conn)
{
    struct netbuf *inbuf = NULL;
    char *buf;
    u16_t buflen;
    err_t err = ERR_OK;

    /* Single threaded server, one context only */
    http_context_t ctx = &server->connection_context;

    /* Initialize context */
    ctx->state = HTTP_PARSING_URI;
    ctx->conn = conn;
    http_parser_init(&ctx->parser, HTTP_REQUEST);
    ctx->parser.data = ctx;
    ctx->server = server;

    const http_parser_settings parser_settings = {
            .on_url = &http_url_cb,
            .on_headers_complete = &http_headers_done_cb,
            .on_header_field = &http_header_name_cb,
            .on_header_value = &http_header_value_cb,
            .on_body = &http_body_cb,
            .on_message_complete = &http_message_done_cb
    };

    while (ctx->state != HTTP_REQUEST_DONE) {
        err = netconn_recv(conn, &inbuf);
        if (err != ERR_OK) {
            break;
        }

        err = netbuf_data(inbuf, (void**) &buf, &buflen);
        if (err != ERR_OK) {
            break;
        }

        size_t parsed_bytes = http_parser_execute(&ctx->parser, &parser_settings, buf, buflen);
        if (parsed_bytes < buflen) {
            break;
        }
    }

    if (err == ERR_OK) {
        ctx->state = HTTP_COLLECTING_RESPONSE_HEADERS;
        if (ctx->handler == NULL) {
            http_send_not_found_response(ctx);
        } else {
            invoke_handler(ctx, HTTP_HANDLE_RESPONSE);
        }
    }

    headers_list_clear(&ctx->request_headers);
    headers_list_clear(&ctx->request_args);

    free(ctx->uri);
    ctx->uri = NULL;
    ctx->handler = NULL;
    if (err != ERR_CLSD) {
        netconn_close(conn);
    }
    if (inbuf) {
        netbuf_delete(inbuf);
    }
}


static void http_server(void *arg)
{
    http_server_t ctx = (http_server_t) arg;
    struct netconn *client_conn;
    err_t err;
    ctx->server_conn = netconn_new(NETCONN_TCP);
    if (ctx->server_conn == NULL) {
        err = ERR_MEM;
        goto out;
    }

    err = netconn_bind(ctx->server_conn, NULL, ctx->port);
    if (err != ERR_OK) {
        goto out;
    }

    err = netconn_listen(ctx->server_conn);
    if (err != ERR_OK) {
        goto out;
    }
    xEventGroupSetBits(ctx->start_done, SERVER_STARTED_BIT);

    do {
        err = netconn_accept(ctx->server_conn, &client_conn);
        if (err == ERR_OK) {
            http_handle_connection(ctx, client_conn);
            netconn_delete(client_conn);
        }
    } while (err == ERR_OK);

out:
    if (ctx->server_conn) {
        netconn_close(ctx->server_conn);
        netconn_delete(ctx->server_conn);
    }
    if (err != ERR_OK) {
        ctx->server_task_err = err;
        xEventGroupSetBits(ctx->start_done, SERVER_DONE_BIT);
    }
    vTaskDelete(NULL);
}

esp_err_t http_server_start(const http_server_options_t* options, http_server_t* out_server)
{
    http_server_t ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->port = options->port;
    ctx->start_done = xEventGroupCreate();
    if (ctx->start_done == NULL) {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    int ret = xTaskCreatePinnedToCore(&http_server, "httpd",
            options->task_stack_size, ctx,
            options->task_priority,
            &ctx->task,
            options->task_affinity);
    if (ret != pdPASS) {
        vEventGroupDelete(ctx->start_done);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    int bits = xEventGroupWaitBits(ctx->start_done, SERVER_STARTED_BIT | SERVER_DONE_BIT, 0, 0, portMAX_DELAY);
    if (bits & SERVER_DONE_BIT) {
        /* Error happened, task is deleted */
        esp_err_t err = lwip_err_to_esp_err(ctx->server_task_err);
        vEventGroupDelete(ctx->start_done);
        free(ctx);
        return err;
    }

    *out_server = ctx;
    return ESP_OK;
}

esp_err_t http_server_stop(http_server_t server)
{
    /* FIXME: figure out a thread safe way to do this */
    netconn_close(server->server_conn);
    xEventGroupWaitBits(server->start_done, SERVER_DONE_BIT, 0, 0, portMAX_DELAY);
    free(server);
    return ESP_OK;
}
