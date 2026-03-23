#include "process/process.hpp"
#include "transformation/transformation.hpp"
#include "barrage/barrage.hpp"
#include "deception/deception.hpp"
#include "process/SimManager.hpp"

#include <thread>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
// #include <fcntl.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <sys/socket.h>
#include <sys/un.h>
#include <csignal>

using namespace Json;

namespace seven {

    // 管道读写缓冲区大小
    const int BUF_SIZE = 4096 * 10000;
    const std::string JSON_DELIMITER = "\n###END###\n";

    volatile bool g_is_running = true;

    // ----------------------------
    // Linux 兼容层（完全兼容 Windows 接口）
    // ----------------------------

    #define INVALID_HANDLE_VALUE -1
    #define NULL 0
    #define TRUE 1
    #define FALSE 0

    inline DWORD GetLastError() { return 0; }

    // 双 Socket 路径（严格对应客户端）
    const char* SOCKET_CMD_PATH  = "/tmp/ClientToServerPipe";
    const char* SOCKET_DATA_PATH = "/tmp/ServerToClientPipe";

    // 控制DOUBLE类型精度
    string formatDouble(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    double formatDouble2(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return std::stod(stream.str());
    }

    std::string jsonToString(const Json::Value& root) {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::string json_str = Json::writeString(builder, root);
        return json_str + JSON_DELIMITER;
    }

    bool stringToJson(const std::string& data, Json::Value& root) {
        size_t delimiter_pos = data.find(JSON_DELIMITER);
        if (delimiter_pos == std::string::npos) {
            std::cerr << "未找到 JSON 数据分隔符" << std::endl;
            return false;
        }
        std::string json_str = data.substr(0, delimiter_pos);

        Json::CharReaderBuilder builder;
        JSONCPP_STRING err;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.size(), &root, &err)) {
            std::cerr << "JSON 解析失败: " << err << std::endl;
            return false;
        }
        return true;
    }

    void parser_cmd(HANDLE hPipe, Json::Value param, Json::Value& result)
    {
        if (param.get("cmd", 4).asInt() == 4) {
            result["status"] = "error";
            result["message"] = "parser command fail";
            result["data"] = Json::nullValue;
            return;
        }
        Sim_Type sim_state = static_cast<Sim_Type>(param.get("sim_type", 4).asInt());
        if (sim_state == Sim_Type::STOPPED) {
            g_sim_manager.sim_stop(result);
        } else if (sim_state == Sim_Type::STARTTED) {
            g_sim_manager.sim_start(param, result);
        } else if (sim_state == Sim_Type::RUNNING) {
            g_sim_manager.sim_calc(hPipe, param, result);
        } else if (sim_state == Sim_Type::ENDDING) {
            g_sim_manager.sim_end(result);
        }
    }

    // bool sendResultData(HANDLE hPipe, const Json::Value& result) {
    //     if (hPipe == INVALID_HANDLE_VALUE) {
    //         std::cerr << "sendResultData: 无效管道" << std::endl;
    //         return false;
    //     }

    //     std::string result_str = jsonToString(result);
    //     if (result_str.empty()) {
    //         std::cerr << "sendResultData: 发送数据为空" << std::endl;
    //         return false;
    //     }

    //     ssize_t ret = write(hPipe, result_str.c_str(), result_str.size());
    //     if (ret <= 0) {
    //         std::cerr << "sendResultData: 发送失败" << std::endl;
    //         return false;
    //     }
    //     return true;
    // }

    // ----------------------------
    // 发送数据（兼容 write，自动适配 socket）
    // ----------------------------
    bool sendResultData(HANDLE fd, const Json::Value& result) {
        if (fd == INVALID_HANDLE_VALUE) {
            std::cerr << "sendResultData: 无效句柄" << std::endl;
            return false;
        }

        std::string result_str = jsonToString(result);
        if (result_str.empty()) {
            std::cerr << "sendResultData: 发送数据为空" << std::endl;
            return false;
        }

        ssize_t ret = send(fd, result_str.c_str(), result_str.size(), MSG_NOSIGNAL);
        if (ret <= 0) {
            std::cerr << "sendResultData: 发送失败" << std::endl;
            return false;
        }
        return true;
    }

    // ----------------------------
    // 双管道：名称完全保持你原来的！
    // SimCalculatorPipe
    // ClientToServerPipe
    // ServerToClientPipe
    // ----------------------------
    // void create_fifo_if_not_exist(const char* path) {
    //     if (access(path, F_OK) == -1) {
    //         if (mkfifo(path, 0666) == 0) {
    //             std::cout << path << " 已创建: " << path << std::endl;
    //         } else {
    //             std::cerr << "创建 "<< path <<" 失败: " << path << " 错误: " << strerror(errno) << std::endl;
    //         }
    //     } else {
    //         std::cout << path << " 已存在 " <<  std::endl;
    //     }
    // }

    // ----------------------------
    // 创建 UNIX Socket 服务端
    // ----------------------------
    int create_unix_socket_server(const char* path) {
        // 确保旧文件删除
        unlink(path);

        int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "socket 创建失败: " << path << std::endl;
            return -1;
        }

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strcpy(addr.sun_path, path);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "bind 失败: " << path << std::endl;
            close(server_fd);
            return -1;
        }

        if (listen(server_fd, 5) < 0) {
            std::cerr << "listen 失败: " << path << std::endl;
            close(server_fd);
            return -1;
        }

        std::cout << "Socket 服务已启动: " << path << std::endl;
        return server_fd;
    }

    // ----------------------------
    // 等待客户端连接
    // ----------------------------
    HANDLE accept_client(HANDLE server_fd) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd >= 0) {
            std::cout << "客户端已连接" << std::endl;
        }
        return client_fd;
    }

    // void handle_client_communication(HANDLE hPipe, HANDLE hPipe_s2c) {
    //     char buffer[4096] = {0};
    //     while (g_is_running) {
    //         ssize_t bytesRead = read(hPipe, buffer, sizeof(buffer)-1);
    //         if (bytesRead <= 0) {
    //             std::cout << "客户端断开" << std::endl;
    //             break;
    //         }

    //         buffer[bytesRead] = '\0';
    //         std::string cmd_str(buffer);
    //         std::cout << "收到命令：" << cmd_str << std::endl;

    //         try {
    //             Json::Value cmd_mes;
    //             if (!stringToJson(cmd_str, cmd_mes)) continue;

    //             Json::Value result;
    //             parser_cmd(hPipe_s2c, cmd_mes, result);
    //             std::string result_str = jsonToString(result);
    //             write(hPipe, result_str.c_str(), result_str.size());
    //         } catch (const std::exception& e) {
    //             Json::Value error_result;
    //             error_result["status"] = "error";
    //             error_result["message"] = std::string("process commande fail：") + e.what();
    //             error_result["data"] = Json::nullValue;
    //             sendResultData(hPipe, error_result);
    //         }
    //         memset(buffer, 0, sizeof(buffer));
    //     }
    // }

    // ----------------------------
    // 处理命令通道（完全不变）
    // ----------------------------
    void handle_client_communication(HANDLE cmd_fd, HANDLE data_fd) {
        char buffer[4096] = {0};

        while (g_is_running) {
            ssize_t n = recv(cmd_fd, buffer, sizeof(buffer)-1, MSG_NOSIGNAL);
            if (n <= 0) {
                std::cout << "命令通道断开" << std::endl;
                break;
            }

            buffer[n] = '\0';
            std::string cmd_str(buffer);
            std::cout << "收到命令: " << cmd_str << std::endl;

            try {
                Json::Value cmd, res;
                if (!stringToJson(cmd_str, cmd)) continue;

                parser_cmd(data_fd, cmd, res);
                std::string resp = jsonToString(res);
                send(cmd_fd, resp.c_str(), resp.size(), MSG_NOSIGNAL);
            }
            catch (const std::exception& e) {
                Json::Value err;
                err["status"] = "error";
                err["message"] = "异常: " + std::string(e.what());
                err["data"] = Json::nullValue;
                sendResultData(cmd_fd, err);
            }
            memset(buffer, 0, sizeof(buffer));
        }
    }

    // void start_pipe_server_test() {
    //     start_pipe_server();
    // }

    // ----------------------------
    // 双管道原版接口 100% 保留
    // 名称：ClientToServerPipe + ServerToClientPipe
    // ----------------------------
    // void start_double_pipe_test() {
    //     const char* PIPE_C2S = "/tmp/ClientToServerPipe";
    //     const char* PIPE_S2C = "/tmp/ServerToClientPipe";

    //     create_fifo_if_not_exist(PIPE_C2S);
    //     create_fifo_if_not_exist(PIPE_S2C);

    //     HANDLE g_hPipe_s2c = INVALID_HANDLE_VALUE;
    //     HANDLE g_hPipe_c2s = INVALID_HANDLE_VALUE;

    //     while (g_is_running) {
    //         g_hPipe_c2s = open(PIPE_C2S, O_RDWR);
    //         if (g_hPipe_c2s >= 0) {
    //             std::cout << "ClientToServerPipe管道已连接" << std::endl;
    //         }

    //         if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
    //             g_hPipe_s2c = open(PIPE_S2C, O_RDWR);
    //             if (g_hPipe_s2c >= 0)
    //             {
    //                 std::cout << "ServerToClientPipe管道已连接" << std::endl;
    //             }
    //         }

    //         if (g_hPipe_c2s >= 0 && g_hPipe_s2c >= 0) {
    //             handle_client_communication(g_hPipe_c2s, g_hPipe_s2c);
    //         }
    //     }

    //     if (g_hPipe_s2c != INVALID_HANDLE_VALUE) {
    //         close(g_hPipe_s2c);
    //         std::cout << "ServerToClientPipe管道已关闭" << std::endl;
    //     }

    //     if (g_hPipe_c2s != INVALID_HANDLE_VALUE) {
    //         close(g_hPipe_c2s);
    //        std::cout << "ClientToServerPipe管道已关闭" << std::endl;
    //     }

    //     std::cout << "管道服务已退出" << std::endl;
    // }

    // ----------------------------
    // 双 UNIX Socket 服务端（替换原来的双FIFO）
    // ----------------------------
    void start_double_pipe_test() {
        // 启动两个 Socket 服务
        int server_cmd  = create_unix_socket_server(SOCKET_CMD_PATH);
        int server_data = create_unix_socket_server(SOCKET_DATA_PATH);

        if (server_cmd < 0 || server_data < 0) {
            std::cerr << "Socket 启动失败" << std::endl;
            return;
        }

        int cmd_fd  = INVALID_HANDLE_VALUE;
        int data_fd = INVALID_HANDLE_VALUE;

        while (g_is_running) {
            // 等待命令通道连接
            if (cmd_fd <= 0) {
                std::cout << "等待命令客户端连接... " << SOCKET_CMD_PATH << std::endl;
                cmd_fd = accept_client(server_cmd);
            }

            // 等待数据通道连接
            if (data_fd <= 0) {
                std::cout << "等待数据客户端连接... " << SOCKET_DATA_PATH << std::endl;
                data_fd = accept_client(server_data);
            }

            // 双连接都成功 → 开始处理
            if (cmd_fd > 0 && data_fd > 0) {
                handle_client_communication(cmd_fd, data_fd);

                // 断开后重置
                close(cmd_fd);
                close(data_fd);
                cmd_fd = INVALID_HANDLE_VALUE;
                data_fd = INVALID_HANDLE_VALUE;
                std::cout << "等待重连..." << std::endl;
            }
        }

        close(server_cmd);
        close(server_data);
        unlink(SOCKET_CMD_PATH);
        unlink(SOCKET_DATA_PATH);

        std::cout << "Socket 服务已退出" << std::endl;
    }

}