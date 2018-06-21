1. ROSマスターを初期化
>> rosinit
2. myAdderROSモデルを開く
3. 右上の「ハードウェアに展開」ボタンをクリックする
4. myAdderROS.tgzが生成される
　 (catkinでビルドするためのpackageファイル)
5. myAdderROS.tgzを解凍すると、
　 myAdderROS.cppというファイルが見つかる
   33行目のmyAdderROS_eML_blk_kernelが
　 アルゴリズムの計算部分になる。
　 ここをGPU Coderで生成したCUDA Cを呼び出す
　　ホストコードに変更する。
