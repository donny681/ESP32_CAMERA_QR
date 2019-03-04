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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @file http_server.h
 * @brief Simple HTTP server
 */

/* Pull in the definitions of HTTP methods */
#include <stdbool.h>
#include "http_parser.h"
#include "esp_err.h"
/**
 * Bit masks for events to be passed to a handler
 */
#define HTTP_HANDLE_URI         BIT(0)      /*!< Called when URI is received */
#define HTTP_HANDLE_HEADERS     BIT(1)      /*!< Called when all headers are received */
#define HTTP_HANDLE_DATA        BIT(2)      /*!< Called each time a fragment of request body is received */
#define HTTP_HANDLE_RESPONSE    BIT(3)      /*!< Called at the end of the request to produce the response */

/** Opaque type representing single HTTP connection */
typedef struct http_context_* http_context_t;

/** Opaque type representing HTTP server */
typedef struct http_server_context_* http_server_t;

/** Callback type of HTTP request handler */
typedef void (* http_handler_fn_t)(http_context_t http_ctx, void* ctx);

/**
 * @brief Configuration of the HTTP server
 */
typedef struct {
    int port;               /*!< TCP Port to listen on */
    int task_affinity;      /*!< Server task affinity (CPU number of tskNO_AFFINITY */
    int task_stack_size;    /*!< Server task stack size, in bytes */
    int task_priority;      /*!< Server task priority */
} http_server_options_t;

/** Default initializer for http_server_options_t */
#define HTTP_SERVER_OPTIONS_DEFAULT()  {\
    .port = 80, \
    .task_affinity = tskNO_AFFINITY, \
    .task_stack_size = 4096, \
    .task_priority = 1, \
}

/**
 * @brief initialize HTTP server, start listening
 * @param options  pointer to http server options, can point to a temporary
 * @param[out] output, handle of the server; pass it to http_server_stop do
 *             delete the server.
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if out of RAM
 *  - ESP_FAIL if some other error
 */
esp_err_t http_server_start(const http_server_options_t* options, http_server_t* out_server);

/**
 * @brief Stop the previously started server
 * @param server handle obtained from http_server_start
 * @return
 *  - ESP_OK on success
 */
esp_err_t http_server_stop(http_server_t server);

/**
 * @brief Register a handler for certain URI
 *
 * The handler will be called when a client makes a request with matching URI
 * and HTTP method.
 *
 * @note Currently only matches full URIs, doesn't support regex
 *
 * @param server  Server handle to register the handler for
 * @param uri_pattern URI pattern to match
 * @param method one of HTTP_GET, HTTP_POST, HTTP_PUT, etc
 * @param events  a bit mask of HTTP_HANDLE_X events for which the handler
 *                should be called
 * @param callback function to call
 * @param callback_arg application context to pass to the callback
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t http_register_handler(http_server_t server, const char* uri_pattern, int method,
                                int events, http_handler_fn_t callback, void* callback_arg);

/**
 * @brief Register a handler for application/x-www-form-urlencoded requests
 *
 * Unlike http_register_handler, handlers registered using this function will
 * not receive HTTP_HANDLE_DATA events. Instead, request body will be parsed
 * into key-value pairs, which can be retrieved while handling
 * HTTP_HANDLE_RESPONSE event using http_request_get_form_value.
 *
 * @param server  Server handle to register the handler for
 * @param uri_pattern URI pattern to match
 * @param method one of HTTP_GET, HTTP_POST, HTTP_PUT, etc
 * @param events  a bit mask of HTTP_HANDLE_X events for which the handler
 *                should be called. HTTP_HANDLE_DATA will not be passed to
 *                the callback.
 * @param callback function to call
 * @param callback_arg application context to pass to the callback
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t http_register_form_handler(http_server_t server, const char* uri_pattern, int method,
                                    int events, http_handler_fn_t callback, void* callback_arg);

/**
 * @brief Get value for given form item name
 * @param http_ctx  context passed to the handler
 * @param name  name of form item
 * @return  pointer to the form item value, valid until the end of request
 */
const char* http_request_get_form_value(http_context_t http_ctx, const char* name);

/**
 * @brief Get request method
 * @param http_ctx  context passed to the handler
 * @return one of HTTP_GET, HTTP_POST, etc
 */
int http_request_get_method(http_context_t http_ctx);

/**
 * @brief Get URI present in the request
 * @param http_ctx  context passed to the handler
 * @return pointer to the URI, valid until the end of request
 */
const char* http_request_get_uri(http_context_t http_ctx);

/**
 * @brief Get request header
 * @param http_ctx  context passed to the handler
 * @param name header name
 * @return
 *  - If the header with given name is present in the request, returns
 *    pointer to the value; valid until request callback returns.
 *  - Otherwise, returns NULL
 */
const char* http_request_get_header(http_context_t ctx, const char* name);

/**
 * @brief Get the event which caused a call to the handler
 * @param ctx  context passed to the handler
 * @return  one of HTTP_HANDLE_X values
 */
int http_request_get_event(http_context_t ctx);

/**
 * @brief Get the request body fragment
 * To be used when handling HTTP_HANDLE_DATA event.
 *
 * @param ctx  context passed to the handler
 * @param[out] out_data_ptr  output, receives pointer to the body fragment
 * @param[out] out_size  output, receives the size of the body fragment
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if called for events other than HTTP_HANDLE_DATA
 */
esp_err_t http_request_get_data(http_context_t ctx, const char** out_data_ptr, size_t* out_size);


/**
 * @brief structure describing a part of the response to be sent
 */
typedef struct {
    const void* data;     /*!< pointer to data to be sent */
    size_t size;    /*!< size of data to send; if data is a 0-terminated string, size can be left 0 */
    bool data_is_persistent;    /*!< set to true if data is in constant RAM */
} http_buffer_t;

#define HTTP_RESPONSE_SIZE_UNKNOWN SIZE_MAX

/**
 * @brief Begin writing HTTP response
 * @param http_ctx  context passed to the handler
 * @param code  HTTP response code
 * @param content_type  string to send as a value in content-type header
 * @param response_size  either the size of the response body, or HTTP_RESPONSE_SIZE_UNKNOWN
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NO_MEM if can't allocate temporary buffer
 *      - other errors from LwIP
 */
esp_err_t http_response_begin(http_context_t http_ctx, int code,
                              const char* content_type, size_t response_size);

/**
 * @brief Add HTTP header to the response
 *
 * This function may be called between http_response_begin and http_response_write.
 *
 * For multipart responses, calling it between http_response_begin and
 * http_response_begin_multipart adds header to the list of headers in the original
 * response (which is sent first). Calling it after http_response_begin_multipart
 * adds the header to the list of headers sent for the current response part.
 *
 * 'name' and 'val' can point to temporary values. This function copies the
 * both strings into internal storage.
 *
 * @param http_ctx  context passed to the handler
 * @param name  Header name
 * @param val   Header value
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_NO_MEM if can't allocate memory for the header
 */
esp_err_t http_response_set_header(http_context_t http_ctx,
                                   const char* name, const char* val);

/**
 * @brief Start one part of a multipart response
 *
 * For multipart responses, the sequence of calls is as follows:
 *
 * 1. http_response_begin (with size == HTTP_RESPONSE_SIZE_UNKNOWN)
 * 2. (optional) http_response_set_header — for the overall response
 * 3. http_response_begin_multipart (if part size is known, pass it in response_size argument)
 * 4. (optional) http_repponse_set_header — for the current part
 * 5. (optional) http_response_write — write response data
 * 6. http_response_end_multipart — when done with the part
 * 7. repeat from 3 as needed
 *
 * @param http_ctx context passed to the handler
 * @param content_type  value to be set in part's content-type header
 * @param response_size  either the size of part body, or HTTP_RESPONSE_SIZE_UNKNOWN
 * @return
 *  - ESP_OK
 *  - ESP_ERR_NO_MEM if can't allocate memory
 *  - other errors from LwIP
 */
esp_err_t http_response_begin_multipart(http_context_t http_ctx,
                                const char* content_type, size_t response_size);

/**
 * @brief Indicate that one part of the multipart response is finished
 * @param http_ctx  context passed to the handler
 * @param boundary  part boundary. Has to be the same as given in the content-type of the first response.
 * @return
 *  - ESP_OK
 *  - other errors from LwIP
 */
esp_err_t http_response_end_multipart(http_context_t http_ctx, const char* boundary);

/**
 * @brief Send a piece of HTTP response to the client
 * @param http_ctx  context passed to the handler
 * @param buffer  data to send, see \ref http_buffer_t
 * @return
 *      - ESP_OK on success
 *      - other errors from LwIP
 */
esp_err_t http_response_write(http_context_t http_ctx, const http_buffer_t* buffer);

/**
 * @brief Indicate that response is complete
 * @param http_ctx  context passed to the handler
 * @return
 *      - ESP_OK on success
 *      - other errors in the future?
 */
esp_err_t http_response_end(http_context_t http_ctx);

#ifdef __cplusplus
}
#endif
