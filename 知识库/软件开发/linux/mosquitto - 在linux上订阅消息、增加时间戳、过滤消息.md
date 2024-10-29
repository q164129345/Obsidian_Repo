## 导言
---
安装
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```

## 一、订阅主题
---
```bash
mosquitto_sub -h 10.64.20.167 -p 1883 -t VCU_LOG
```
这里的参数解释如下：
- `h` 指定MQTT服务器的主机名或IP地址。
- `t` 指定要订阅的主题。
如果你的MQTT服务器配置了端口、用户名和密码等其他设置，你可能还需要添加相关的命令行参数，例如：
- `p` 指定服务器端口（默认1883）。
- `u` 指定用户名。
- `P` 指定密码。

稍微改进一下，增加时间戳的打印。
```bash
mosquitto_sub -h 10.64.20.167 -p 1883 -t VCU_LOG | while IFS= read -r line; do
  echo "$(date '+%Y-%m-%d %H:%M:%S') $line"
done

```

- 使用 `while IFS= read -r line` 读取每一行输出。这里的 `IFS=` 保证了行首行尾的空白字符不会被剥离，`r` 参数防止对反斜线的特殊处理。
- 对于每一行输出，`echo "$(date '+%Y-%m-%d %H:%M:%S') $line"` 将在行前加上当前的时间戳。
![[Pasted image 20241029170108.png]]

## 二、增加时间戳，且增加内容筛选
---
### 安装 `moreutils` 和 `xxd`
如果 `ts` 命令不可用，可以通过以下命令安装 moreutils 包：
```bash
sudo apt update
sudo apt install moreutils
```

`xxd` 通常是作为 `vim-common` 包的一部分安装的。如果没有安装，可以通过以下命令安装：
```bash
sudo apt install vim-common
```

这个shell代码的目的是打印MQTT消息：
```bash
mosquitto_sub -h 10.64.21.37 -p 1883 -t VCU_STAT | while read -r line
do
  # 转换为十六进制字符串
  hex_line=$(echo -n "$line" | xxd -p -c 256 | sed 's/\\(..\\)/\\1 /g')

  # 提取前四个字节
  prefix=$(echo "$hex_line" | cut -d' ' -f1-4)

  # 过滤报文，精确匹配 "aa 01 20 01" 开头的报文
  if [[ "$prefix" == "aa 01 20 01" ]]; then
    echo "$(date +%Y-%m-%d\\ %H:%M:%S) $hex_line"
  fi
done

```

- 有时间戳
- MQTT主题：VCU_STAT
- 内容格式：十六进制
- 筛选：消息的开头是aa012001

效果如下：
![[Pasted image 20241029170128.png]]