# robot
cell to cell algorithm


![image](https://github.com/WeJay/robot/blob/master/png/flow.JPG)



實線是牆壁，虛線是虛擬框出的cell。   
    
起點為start，一開始編號和cell均為未知，出發後第一個cell編號是1 ，並且得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{12},y_{12})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{12},y_{12})" title="(x_{12},y_{12})" /></a>為從1跨到2的點，產生編號2的cell。    
    
走到編號2的cell，得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{23},y_{23})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{23},y_{23})" title="(x_{23},y_{23})" /></a>產生編號3的cell、接著得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{24},y_{24})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{24},y_{24})" title="(x_{24},y_{24})" /></a>產生編號4的cell。Ps:”接著得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{21},y_{21})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{21},y_{21})" title="(x_{21},y_{21})" /></a>由於這是剛剛跨過來的這個邊界因此不用產生新的cell”。   
    
再來從編號2的cell走到編號3的cell，相同的得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{35},y_{35})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{35},y_{35})" title="(x_{35},y_{35})" /></a>產生編號5的cell，得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{36},y_{36})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{36},y_{36})" title="(x_{36},y_{36})" /></a>產生編號6的cell，得到<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{37},y_{37})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{37},y_{37})" title="(x_{37},y_{37})" /></a>產生編號7的cell。 Ps:”得到點<a href="https://www.codecogs.com/eqnedit.php?latex=(x_{32},y_{32})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x_{32},y_{32})" title="(x_{32},y_{32})" /></a>由於這是剛剛跨過來的這個邊界因此不用產生新的cell”。   
    
    
![image](https://github.com/WeJay/robot/blob/master/png/flow2.JPG)
	
	
	
![image](https://github.com/WeJay/robot/blob/master/png/graph.JPG)