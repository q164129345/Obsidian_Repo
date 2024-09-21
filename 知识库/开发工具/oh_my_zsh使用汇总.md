# 一、主题
---
## 1.1、af-magic
![[截屏2024-09-21 22.35.23.png]]
这个主题跟终端终结者一起使用，很爽。
![[截屏2024-09-21 22.36.37.png]]

# 二、插件
---
1、zsh-autosuggestions（补全建议）
```bash
git clone <https://github.com/zsh-users/zsh-autosuggestions> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
```
2、zsh-syntax-highlighting （高亮）
```bash
git clone <https://github.com/zsh-users/zsh-syntax-highlighting.git> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```
3、zsh-completions （自动补全）
```bash
git clone <https://github.com/zsh-users/zsh-completions> ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions
```
## 2.1、修改~/.zshrc
---
```jsx
# 找到plugins，填入插件的名字
plugins=(   git
            zsh-autosuggestions
            zsh-syntax-highlighting
            zsh-completions
        )
```

最后，调用终端，输入以下指令激活一下新的配置文件。
```bash
source ~/.zshrc
```

