# robot

participant
participant1: 学習者の左手
participant2: 学習者の右手
participant3: 熟練者の左手
participant4: 熟練者の右手

operation_mode
1: モーションキャプチャーのみでロボットが動作
2: 記録データのみでロボットが動作
3: 変動融合割合でロボットが動作
4: マニュアル融合割合でロボットが動作（settings.jsonのweightListPos、weightListRotで指定）
5: ロボットが動作しない
6: circkeAgent用割合（agentが起動したときに順に割合を変化させて0.5にする）

eulerOrder
1: 鉗子＆陶芸
2: 陶芸　左向き？

armMode
xArm1: 鉗子用
xArm2: 陶芸
xArm3: circleAgent用　