# nhk2025b_ros2
ros2のwsです

## 準備
1. ROS 2 humble をインストール
    - Windowsの場合、WSLでUbuntu22.04をインストール後humbleインストール
    - Dockerが利用可能な場合、VSCodeのDevContainerを使って環境構築可能
2. リポジトリをクローン
    - もしNHK2025Bを作成していない場合
    ```bash
    mkdir ~/NHK2025B
    ```
    ```bash
    cd ~/NHK2025B
    git clone git@github.com:NITIC-Robot-Club/nhk2025b_ros2.git
    ```
3. ビルド手順
    ```bash
    cd ~/NHK2025B/nhk2025b_ros2
    colcon build
    ```

## 開発
1. Issueを作成
    - どんな作業をやるのかを明記
    - 以下の接頭語をつけること
        - feat：新規機能など
        - fix：バグ修正
    - 例
        - feat: nhk2025b_newpackage
        - feat(nhk2025b_newpackage): add new feat
        - fix(nhk2025b_newpackage): fix zero division
2. 要件等をまとめる
    - どんな入出力なのか
    - 何を変更するのか
    - そのバグはどのような状況で再現できるか
    - どんな修正を含むのか
3. ローカルで新しくブランチを作成
    - Issueを開き、右下のDevelopmentの`create branch`を押して自動生成する
    - この操作により、リモートリポジトリに新しいブランチが作成されます
    - 表示されたコマンドをローカルで実行して、新しいブランチをチェックアウトしてください
        ```bash
        git fetch origin
        git checkout ブランチ名
        ```
4. 変更を加える
    - コミットメッセージは最終的にはまとめられるが、わかりやすいものがいい
    - コミットメッセージに規則はない
    - auto lintが働いてコードがきれいになります
    - 作業前にpullを忘れずに！
    - 変更が終わったらpushする
    ```bash
    git push origin ブランチ名
    ```
5. PRを作成する
    - githubでリポジトリを開き、作成したブランチを開く
    - Compare & pull request ボタンを押してPRを作成する
    - PRのタイトルはIssueと同じにする
6. コードレビュー&マージ
    - kazu-321 にコードレビューを割り当てる
    - 時間のあるときにコードの検証等を行うのでマージされるまで待つ
    - 3日経ってもマージされない場合はdiscordで連絡

## ROS 2 テンプレ
### package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>ーーーパッケージ名ーーー</name>
  <version>0.0.1</version>
  <description>ーーー説明ーーー</description>
  <maintainer email="ーーーメアドーーー">ーーー名前ーーー</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>ーーー依存ーーー</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt
```c
cmake_minimum_required(VERSION 3.8)
project(ーーーパッケージ名ーーー)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "ネームスペース::クラス名"
    EXECUTABLE ノード名
)

ament_auto_package(
    INSTALL_TO_SHARE
)
```

### include/パッケージ名/ノード名.hpp
```cpp
#ifndef __ノード名_HPP__
#define __ノード名_HPP__

#include <rclcpp/rclcpp.hpp>
// #include <test_msgs/msg/my_message.hpp>

namespace ネームスペース名 {
    class クラス名 : public rclcpp::Node {
    public:
        クラス名(const rclcpp::NodeOptions & options);
    private:
        // コールバック関数とか変数とか
        // void 受信時に呼び出す関数名(const test_msgs::msg::MyMessage::SharedPtr msg);
        // void タイマーで呼び出す関数名();
        // rclcpp::Publisher<test_msgs::msg::MyMessage>::SharedPtr パブリッシャー;
        // rclcpp::Subscription<test_msgs::msg::MyMessage>::SharedPtr サブスクライバー;
        // rclcpp::TimerBase::SharedPtr タイマー;
    };
}

#endif//__ノード名_HPP__
```

### src/ノード名.cpp
```cpp
#include "パッケージ名/ノード名.hpp"

namespace ネームスペース名 {
    クラス名::クラス名(const rclcpp::NodeOptions & options)
        : Node("ノード名",options) {
            // パブリッシャー = this->create_publisher<test_msgs::msg::MyMessage>("topic名", 10);
            // サブスクライバー = this->create_subscription<test_msgs::msg::MyMessage>("topic名", 10, std::bind(&クラス名::受信時に呼び出す関数名, this, std::placeholders::_1));
            // タイマー = this->create_wall_timer(std::chrono::milliseconds(周期), std::bind(&クラス名::タイマーで呼び出す関数名, this));
        }

        // void クラス名::受信時に呼び出す関数名(const test_msgs::msg::MyMessage::SharedPtr msg) {
        //     msg->test のように->を使って情報にアクセス
        // }

        // void タイマーで呼び出す関数名() {
        //     定期的に実行
        // }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ネームスペース名::クラス名)
```