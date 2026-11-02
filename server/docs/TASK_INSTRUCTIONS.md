# 采集英文指令模板

十种正式任务各 20 条模板，共 200 条。编辑 `config/task-instructions.json` 修改句子。每条尝试用独立 `task-instruction-v1` 随机流均匀、有放回地选择一句，因此连续两条可能选择同一模板。模板同时包含简短和详细表述，目标动作保持一致。

插头/螺栓模板中的 `{source_color}` 和 `{target_color}` 使用本条实际源件及目标孔配色。颜色不改变任务物理对应关系。抽屉模板保留约 8 cm 开合目标，完整锁杆任务保留松开、放倒入卡扣、扶起、重新拧紧顺序。四个单向任务只描述所需的方向与对应初态，不把反向动作写入指令；既有六种任务的模板与随机流不变。

选择保存于 `collection_config.instruction_randomization`：种子、模板编号、实际模板、SHA256、最终 instruction；录制任务元数据同步保存该记录。重复准备和回放复用所存模板，新批次逐次重新选择；改变模板库后需重启采集进程。新增随机流不会改变布局、关节、机位、配色的随机结果。

以下是全部模板；示例统一用 red 源件和 blue 目标孔，实际采集按真实颜色填入。

## 抽屉开合 · `drawer_cycle`

1. Open the drawer about 8 cm, close it fully, and release it.
2. Pull the drawer out about 8 cm, then close it completely and let go.
3. Extend the drawer approximately 8 cm, return it fully, and release it.
4. Open the drawer roughly 8 cm, then shut it fully and let go.
5. Pull out the drawer by about 8 cm and push it fully closed before releasing it.
6. Move the drawer out approximately 8 cm, close it completely, and let go.
7. Open the drawer to about 8 cm, return it to the closed position, and release it.
8. Pull the drawer open approximately 8 cm, then close it fully and release your grip.
9. Grasp the drawer, open it about 8 cm, then close it fully and release it.
10. Open the drawer approximately 8 cm, then close it fully and release it.
11. Pull the drawer out roughly 8 cm, guide it back until fully closed, and let go.
12. Take hold of the drawer and open it about 8 cm. Close it completely before releasing it.
13. Grasp the drawer and pull it open approximately 8 cm, then push it fully closed and release your grip.
14. Open the drawer by about 8 cm and return it to its fully closed position. Release it and move the gripper away.
15. Pull out the drawer approximately 8 cm, push it back until fully closed, then release it and withdraw the gripper.
16. Use the gripper to open the drawer about 8 cm. Return the drawer fully to its closed position and let go.
17. Grasp the drawer, pull it out roughly 8 cm, and guide it back to the closed position. Once fully closed, release it.
18. Open the drawer approximately 8 cm while holding it with the gripper, then close it all the way, release it, and move clear.
19. Take hold of the drawer, open it by about 8 cm, and return it fully to its original closed position. Let go and withdraw the gripper.
20. Approach and grasp the drawer, pull it open approximately 8 cm, then guide it back until fully closed. Release the drawer and move the gripper clear.

## 航空插头 · `aviation_plug_insert`

1. Insert the red aviation plug fully into the blue socket.
2. Place the red aviation plug all the way into the blue socket.
3. Fit the red aviation plug fully into the blue socket.
4. Move the red aviation plug to the blue socket and insert it fully.
5. Pick up the red aviation plug and seat it fully in the blue socket.
6. Take the red aviation plug and insert it all the way into the blue socket.
7. Connect the red aviation plug to the blue socket, seating it fully.
8. Grasp the red aviation plug, then insert it fully into the blue socket.
9. Align the red aviation plug with the blue socket and insert it fully.
10. Transfer the red aviation plug into the blue socket until it is fully seated.
11. Pick up the red aviation plug, align it with the blue socket, and insert it fully.
12. Remove the red aviation plug from its starting socket and insert it fully into the blue socket.
13. Lift the red aviation plug from its holder, move it over the blue socket, and insert it fully.
14. Grasp the red aviation plug, move it to the blue socket, and release it once fully inserted.
15. Bring the red aviation plug to the blue socket, align the plug, and seat it completely.
16. Pick up the red aviation plug and position it above the blue socket. Align it and insert it fully.
17. Take the red aviation plug out of its holder, transfer it to the blue socket, and seat it fully before letting go.
18. Find the red aviation plug and the blue socket. Pick up the plug, align it with that socket, and insert it all the way.
19. Move the red aviation plug from its starting position to the blue socket. Insert it fully, release the plug, and move the gripper clear.
20. Grasp and lift the red aviation plug, carry it above the blue socket, align it with the opening, and insert it fully. Release it and withdraw the gripper.

## 导向插头 · `guide_plug_insert`

1. Insert the red guide plug fully into the blue socket.
2. Place the red guide plug all the way into the blue socket.
3. Fit the red guide plug fully into the blue socket.
4. Move the red guide plug to the blue socket and insert it fully.
5. Pick up the red guide plug and seat it fully in the blue socket.
6. Take the red guide plug and insert it all the way into the blue socket.
7. Connect the red guide plug to the blue socket, seating it fully.
8. Grasp the red guide plug, then insert it fully into the blue socket.
9. Align the red guide plug with the blue socket and insert it fully.
10. Transfer the red guide plug into the blue socket until it is fully seated.
11. Pick up the red guide plug, align it with the blue socket, and insert it fully.
12. Remove the red guide plug from its starting socket and insert it fully into the blue socket.
13. Lift the red guide plug from its holder, move it over the blue socket, and insert it fully.
14. Grasp the red guide plug, move it to the blue socket, and release it once fully inserted.
15. Bring the red guide plug to the blue socket, align the plug, and seat it completely.
16. Pick up the red guide plug and position it above the blue socket. Align it and insert it fully.
17. Take the red guide plug out of its holder, transfer it to the blue socket, and seat it fully before letting go.
18. Find the red guide plug and the blue socket. Pick up the plug, align it with that socket, and insert it all the way.
19. Move the red guide plug from its starting position to the blue socket. Insert it fully, release the plug, and move the gripper clear.
20. Grasp and lift the red guide plug, carry it above the blue socket, align it with the opening, and insert it fully. Release it and withdraw the gripper.

## 圆柱插头 · `cylindrical_plug_insert`

1. Insert the red cylindrical plug fully into the blue socket.
2. Place the red cylindrical plug all the way into the blue socket.
3. Fit the red cylindrical plug fully into the blue socket.
4. Move the red cylindrical plug to the blue socket and insert it fully.
5. Pick up the red cylindrical plug and seat it fully in the blue socket.
6. Take the red cylindrical plug and insert it all the way into the blue socket.
7. Connect the red cylindrical plug to the blue socket, seating it fully.
8. Grasp the red cylindrical plug, then insert it fully into the blue socket.
9. Align the red cylindrical plug with the blue socket and insert it fully.
10. Transfer the red cylindrical plug into the blue socket until it is fully seated.
11. Pick up the red cylindrical plug, align it with the blue socket, and insert it fully.
12. Remove the red cylindrical plug from its starting socket and insert it fully into the blue socket.
13. Lift the red cylindrical plug from its holder, move it over the blue socket, and insert it fully.
14. Grasp the red cylindrical plug, move it to the blue socket, and release it once fully inserted.
15. Bring the red cylindrical plug to the blue socket, align the plug, and seat it completely.
16. Pick up the red cylindrical plug and position it above the blue socket. Align it and insert it fully.
17. Take the red cylindrical plug out of its holder, transfer it to the blue socket, and seat it fully before letting go.
18. Find the red cylindrical plug and the blue socket. Pick up the plug, align it with that socket, and insert it all the way.
19. Move the red cylindrical plug from its starting position to the blue socket. Insert it fully, release the plug, and move the gripper clear.
20. Grasp and lift the red cylindrical plug, carry it above the blue socket, align it with the opening, and insert it fully. Release it and withdraw the gripper.

## 航空螺栓 · `aviation_bolt_insert`

1. Unscrew the red aviation bolt, then screw it fully into the blue hole.
2. Remove the red aviation bolt by unscrewing it, then screw it fully into the blue hole.
3. Unscrew the red aviation bolt from its holder and screw it fully into the blue hole.
4. Take out the red aviation bolt by unscrewing it and screw it all the way into the blue hole.
5. Unscrew and remove the red aviation bolt, then install it fully in the blue hole by screwing it in.
6. Turn the red aviation bolt out of its holder, then screw it fully into the blue hole.
7. Unscrew the red aviation bolt, move it to the blue hole, and screw it all the way in.
8. Grasp the red aviation bolt, unscrew it, and screw it fully into the blue hole.
9. Unscrew the red aviation bolt from its starting hole and screw it fully into the blue hole.
10. Unscrew the red aviation bolt from its holder, then screw it fully into the blue hole.
11. Remove the red aviation bolt from its holder by unscrewing it. Move it to the blue hole and screw it fully in.
12. Grasp and unscrew the red aviation bolt, align it with the blue hole, and screw it in fully.
13. Unscrew the red aviation bolt until it can be removed, then transfer it to the blue hole and screw it fully in.
14. Take hold of the red aviation bolt and unscrew it from its holder. Align it with the blue hole and screw it all the way in.
15. Unscrew and lift the red aviation bolt from its holder, move it above the blue hole, and screw it fully into place.
16. Find the red aviation bolt and unscrew it from its starting hole. Carry it to the blue hole and screw it fully in.
17. Grasp the red aviation bolt, unscrew and remove it, then align it with the blue hole. Screw it fully in before releasing it.
18. Unscrew the red aviation bolt from its holder and transfer it to the blue hole. Screw it all the way in, release it, and move clear.
19. Remove the red aviation bolt by unscrewing it from its holder, carry it to the blue hole, align the bolt, and screw it fully in. Release it afterward.
20. Approach and grasp the red aviation bolt, unscrew it from its holder, and lift it clear. Move it to the blue hole, screw it fully in, then release it and withdraw the gripper.

## 锁杆放倒与复位 · `locking_rod_cycle`

1. Loosen the fastener, fold the locking rod into its clip, then raise the rod and retighten the fastener.
2. Loosen the locking rod's fastener, fold the rod into its clip, then raise it and tighten the fastener.
3. Loosen the fastener and lower the locking rod into its clip. Raise the rod again and retighten the fastener.
4. Loosen the locking rod's fastener, fold the rod into the clip, then return the rod upright and retighten the fastener.
5. Release the locking rod by loosening its fastener, fold the rod into its clip, then raise and secure it by tightening the fastener.
6. Loosen the fastener, lower the locking rod into the clip, then lift the rod back up and tighten the fastener.
7. Fold the locking rod into its clip after loosening the fastener, then raise the rod and retighten the fastener.
8. Loosen the locking rod's fastener and fold the rod into the clip. Return the rod upright and tighten its fastener.
9. Loosen the fastener to fold the locking rod into its clip, then restore the rod upright and tighten the fastener again.
10. Loosen the fastener, fold the locking rod down into its clip, then raise the rod back to its starting position and retighten the fastener.
11. Grasp and loosen the locking rod's fastener, fold the rod into its clip, then raise the rod and tighten the fastener again.
12. Loosen the locking rod's fastener and lower the rod into the clip. Release it, grasp it again, raise it, and retighten the fastener.
13. First loosen the fastener and fold the locking rod into its clip. Then raise the rod back upright and secure it by tightening the fastener.
14. Take hold of the locking rod's fastener and loosen it. Fold the rod into the clip, then raise it to its original position and retighten the fastener.
15. Loosen the fastener, fold the locking rod into its clip, and let go. Grasp the rod again, raise it upright, and tighten the fastener.
16. Loosen the locking rod's fastener before folding the rod into the clip. After releasing it in the clip, grasp it again, raise it, and retighten the fastener.
17. Fold the locking rod down into its clip after loosening its fastener. Release the rod, take hold of it again, return it upright, and tighten the fastener.
18. Grasp the locking rod's fastener and loosen it, then lower the rod into the clip and release it. Grasp the rod again, raise it upright, retighten the fastener, and let go.
19. Loosen the fastener so the locking rod can fold into its clip. Release the folded rod, grasp it again, return it to its upright position, and tighten the fastener before releasing it.
20. Approach the locking rod and loosen its fastener, fold the rod into its clip, and release it. Grasp the rod again, raise it to its original position, tighten the fastener, then release it and move the gripper clear.

## 抽屉拉起 · `drawer_open`

1. Open the drawer about 8 cm and release it.
2. Pull the drawer open approximately 8 cm and let go.
3. Lift the drawer about 8 cm and leave it open.
4. Extend the drawer approximately 8 cm, then release your grip.
5. Pull out the drawer roughly 8 cm and leave it there.
6. Raise the drawer about 8 cm, then release it.
7. Open the closed drawer approximately 8 cm and let go.
8. Grasp the drawer and pull it open about 8 cm.
9. Move the drawer out about 8 cm and release it.
10. Open the drawer to approximately 8 cm and move clear.
11. Grasp the closed drawer, open it about 8 cm, then release it.
12. Take hold of the drawer and lift it approximately 8 cm. Leave it open.
13. Pull the drawer out roughly 8 cm, release your grip, and withdraw the gripper.
14. Approach the drawer, grasp its handle, and open it approximately 8 cm before letting go.
15. Use the gripper to release the drawer latch and open the drawer about 8 cm. Let go afterward.
16. Grasp the handle of the closed drawer and pull it open approximately 8 cm. Release it and move away.
17. Open the drawer by about 8 cm while holding its handle, then release the handle and withdraw the gripper.
18. Take hold of the closed drawer, release its latch, and lift it about 8 cm. Leave the drawer open and move clear.
19. Approach and grasp the drawer handle, pull the drawer open approximately 8 cm, then release it and move the gripper clear.
20. With the drawer initially closed, grasp its handle, release the latch, and open it by about 8 cm. Let go and withdraw the gripper.

## 抽屉放回 · `drawer_close`

1. Close the drawer from its 8 cm open position and release it.
2. Push the drawer, initially open about 8 cm, fully closed.
3. Return the drawer from about 8 cm open to fully closed.
4. Close the drawer that is open approximately 8 cm and let go.
5. Lower the drawer from its 8 cm open position and latch it.
6. Push the 8 cm open drawer fully back and release it.
7. Grasp the drawer opened about 8 cm and close it fully.
8. Shut the drawer from approximately 8 cm open and release your grip.
9. Return the drawer opened about 8 cm to its closed position.
10. Close and latch the drawer from about 8 cm open, then release it.
11. Grasp the drawer, initially open about 8 cm, and guide it fully closed before letting go.
12. Take hold of the drawer that is open about 8 cm, return it fully, and release it.
13. Push the drawer from its approximately 8 cm open position until fully closed, then withdraw the gripper.
14. Approach the drawer opened about 8 cm, grasp its handle, and return it to the closed position.
15. Use the gripper to close the drawer from about 8 cm open. Make sure it is latched before releasing it.
16. Grasp the handle of the drawer that is open approximately 8 cm, guide it fully closed, then release and move clear.
17. Return the drawer from its 8 cm open position to fully closed while holding its handle, then release the handle and withdraw.
18. Take hold of the drawer initially open about 8 cm, lower it until closed and latched, then let go and move away.
19. Approach and grasp the drawer handle while the drawer is open approximately 8 cm. Guide it fully closed, release it, and withdraw.
20. With the drawer initially open about 8 cm, grasp its handle and return it fully to its closed position. Engage the latch, let go, and move clear.

## 锁杆装置放倒 · `locking_rod_fold`

1. Loosen the fastener and fold the locking rod into its clip.
2. Fold the locking rod into its clip after loosening the fastener.
3. Loosen the locking rod fastener, then lower the rod into its clip.
4. Unscrew the fastener enough to fold the locking rod into its clip.
5. Release the locking rod fastener and fold the rod into its clip.
6. Loosen the fastener, fold the locking rod into the clip, and let go.
7. Lower the locking rod into its clip after releasing its fastener.
8. Grasp the fastener, loosen it, and fold the locking rod into its clip.
9. Loosen the locking rod fastener and place the rod in its retaining clip.
10. Fold down the locking rod into its clip once its fastener is loose.
11. Grasp and loosen the locking rod fastener, then lower the rod into its clip and release it.
12. Take hold of the locking rod fastener and loosen it. Fold the rod into its clip and let go.
13. Loosen the fastener holding the locking rod upright, lower the rod into its clip, then withdraw the gripper.
14. Approach the locking rod, engage its fastener with the gripper, and loosen it before folding the rod into its clip.
15. Use the gripper to loosen the locking rod fastener, then guide the rod downward until it sits in its clip.
16. Grasp the locking rod fastener and loosen it fully enough to release the rod. Fold the rod into its clip, then let go.
17. With the locking rod initially upright, loosen its fastener and lower the rod into its retaining clip. Release it and move clear.
18. Approach and grasp the locking rod fastener, rotate it loose, then fold the rod down into the clip before opening and withdrawing the gripper.
19. Loosen the fastener securing the upright locking rod, regrasp for folding, and guide the rod into its clip. Leave it folded and move the gripper away.
20. With the locking rod upright and secured, grasp and loosen the fastener, fold the rod into its retaining clip, then release the rod and withdraw the gripper.

## 锁杆装置复位 · `locking_rod_restore`

1. Raise the locking rod from its clip and tighten the fastener.
2. Restore the locking rod from its clip and secure its fastener.
3. Lift the locking rod from the clip, then tighten its fastener.
4. Return the locking rod upright from its clip and tighten the fastener.
5. Raise the folded locking rod out of its clip and secure the fastener.
6. Take the locking rod out of its clip and tighten its fastener upright.
7. Lift the locking rod from its clip and lock it with the fastener.
8. Grasp the locking rod in its clip, raise it, and tighten its fastener.
9. Restore the locking rod upright from the clip, tightening its fastener.
10. Raise the locking rod from the clip, tighten the fastener, and release it.
11. Grasp the folded locking rod, lift it out of its clip, and tighten the fastener once upright.
12. Take hold of the locking rod resting in its clip. Raise the rod and secure it by tightening the fastener.
13. Lift the locking rod from its retaining clip, guide it back upright, tighten the fastener, and withdraw the gripper.
14. Approach the folded locking rod, grasp it in the clip, and raise it upright before tightening its fastener.
15. Use the gripper to free the locking rod from its clip, raise it to its upright position, and tighten the fastener.
16. Grasp the locking rod resting in its clip, lift it upright, then engage and tighten the fastener before releasing it.
17. With the locking rod initially folded into its clip, raise it back upright and tighten its fastener. Release it and move clear.
18. Approach and grasp the locking rod in its clip, raise it to its original upright position, tighten the fastener, then open and withdraw the gripper.
19. Remove the folded locking rod from its clip, guide it upright, and regrasp its fastener to tighten it. Leave the rod secured and move away.
20. With the locking rod folded into its retaining clip, grasp and raise it upright, align the fastener, tighten it securely, then release and withdraw the gripper.
