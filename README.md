# nhk2025b_ros2
ros2のwsです

## 準備
1. ROS 2 humble をインストール
    - Windowsの場合、WSLでUbuntu22.04をインストール後humbleインストール
    - Dockerが利用可能な場合、VSCodeのDevContainerを使って環境構築可能
2. リポジトリをクローン
    もしNHK2025Bを作成していない場合
    ```bash
    mkdir ~/NHK2025B
    ```
    ```bash
    cd ~/NHK2025B
    git clone git@github.com:NITIC-Robot-Club/nhk2025b_ros2.git
    ```
3. ビルド手順
    ```bash
    cd NHK2025B/nhk2025b_ros2
    . colcon_build.sh
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
    - ブランチ名はIssue番号を使う
    - 例
        - feat/#0_nhk2025b_newpackage
        - feat/#1_nhk2025b_newpackage_add_new_feat
        - fix/#2_nhk2025b_newpackage_fix_zero_division
    - コマンド
        ```bash
        git branch ブランチ名
        git checkout ブランチ名
        ```
4. 変更を加える
    - コミットメッセージは最終的にはまとめられるが、わかりやすいものがいい
    - コミットメッセージに規則はない
    - 変更が終わったらpushする
    ```bash
    git push origin ブランチ名
    ```
5. PRを作成する
    - githubでリポジトリを開き、作成したブランチを開く
    - Compare & pull request ボタンを押してPRを作成する
    - PRのタイトルはIssueと同じにする
    - PRの説明文にはIssueのURLを載せておく
6. コードレビュー&マージ
    - kazu-321 にコードレビューを割り当てる
    - 時間のあるときにコードの検証等を行うのでマージされるまで待つ
    - 3日経ってもマージされない場合はdiscordで連絡