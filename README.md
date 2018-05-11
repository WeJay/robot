# robot
cell to cell algorithm


![image](https://github.com/WeJay/robot/blob/master/png/flow.JPG)



實線是牆壁，虛線是虛擬框出的cell。   
    
起點為start，一開始編號和cell均為未知，出發後第一個cell編號是1 ，並且得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{12},y_{12})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{12},y_{12})" title="(x_{12},y_{12})" /></a>為從1跨到2的點，產生編號2的cell。    
    
走到編號2的cell，得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{23},y_{23})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{23},y_{23})" title="(x_{23},y_{23})" /></a>產生編號3的cell、接著得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{24},y_{24})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{24},y_{24})" title="(x_{24},y_{24})" /></a>產生編號4的cell。Ps:”接著得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{21},y_{21})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{21},y_{21})" title="(x_{21},y_{21})" /></a>由於這是剛剛跨過來的這個邊界因此不用產生新的cell”。   
    
再來從編號2的cell走到編號3的cell，相同的得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{35},y_{35})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{35},y_{35})" title="(x_{35},y_{35})" /></a>產生編號5的cell，得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{36},y_{36})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{36},y_{36})" title="(x_{36},y_{36})" /></a>產生編號6的cell，得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{37},y_{37})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{37},y_{37})" title="(x_{37},y_{37})" /></a>產生編號7的cell。 Ps:”得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{32},y_{32})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{32},y_{32})" title="(x_{32},y_{32})" /></a>由於這是剛剛跨過來的這個邊界因此不用產生新的cell”。   
    
    
![image](https://github.com/WeJay/robot/blob/master/png/flow2.JPG)
	
	
	
![image](https://github.com/WeJay/robot/blob/master/png/graph.JPG)



把所有突破口存到crosspoint[100]		
		
紀錄和判斷每次wall_follow第一個突破口(crosspoint[first_bump])		
		
紀錄每次wall_follow的起始點(stop_laser == 1 | first_bump != 0)		

若沒有碰撞點則”直接從頭搜尋”最起始的突破口或者”紀錄上次搜尋到哪當作搜尋的起點”(first_bump == 0)		

判斷剛跨過的突破口不紀錄!(記錄跨cell的邊界)(LastCrossBoundary)		

跨出cell的行為,找牆wall_follow 或者 直直衝到邊界		

處理不同突破口進到同樣的cell		

cell to cell algorithm(walk sequence non-sorting)		

1. record and decide first cross point when wall_follow once(finish)		

2. If there isn't new cross point,searching cross point from start or from index searched last time in crosspoint array.(finish)		

3. not record cross point at last cross boundary.(finish)		

4. when cross cell,finding the nearest wall to wall_follow or go straight until it is out the boundary.(finish)		

5. If differnt boundary produce different cross point, but they enter the same cell.(no solution so far)		
