#include "astar.h"
#include <float.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

int direction[8][2] = {
	{0,-1},//上
	{0,1},//下
	{-1,0},//左
	{1,0},//右
	{-1,-1},//左上
	{1,-1},//右上
	{-1,1},//左下
	{1,1},//右下
};

bool AStar::Init(int x,int y,std::map<std::pair<int,int>,int> &values){
	
	if(x < 0 || y < 0 || x*y != values.size()) 
		return false;
	m_xcount = x;
	m_ycount = y;
	m_map = (mapnode*)calloc(m_xcount*m_ycount,sizeof(*m_map));
	int i = 0;
	int j = 0;
	int c = 0;
	for( ; i < m_ycount; ++i)
	{
		for(j = 0; j < m_xcount;++j)
		{		
			mapnode *tmp = &m_map[i*m_xcount+j];
			tmp->x = j;
			tmp->y = i;
			
			std::pair<int,int> idx = std::make_pair(tmp->x,tmp->y);
			tmp->value = values[idx];
		}
	}
	return true;
}

void AStar::reset(){
	//清理close list
	mapnode *n = NULL;
	while(n = (mapnode*)close_list.Pop()){
		n->G = n->H = n->F = 0;
	}
	//清理open list
	open_list.Clear();
}

bool AStar::find_path(int x,int y,int x1,int y1,std::list<mapnode*> &path)
{
	mapnode *from = get_mapnode(x,y);
	mapnode *to = get_mapnode(x1,y1);
	if(from == to || to->value == 0xFFFFFFFF){
		path.push_back(from);		
		return true;
	}
	open_list.insert(from);	
	mapnode *current_node = NULL;	
	while(1){	
		current_node = (mapnode*)open_list.popmin();
		if(!current_node){ 
			reset();
			return false;
		}
		if(current_node == to){ 
			
			while(current_node)
			{
				path.push_front(current_node);
				mapnode *t = current_node;
				current_node = current_node->parent;
				t->parent = NULL;
				t->F = t->G = t->H = 0;
				t->index = 0;
			}	
			reset();
			return true;
		}
		//current插入到close表
		close_list.Push(current_node);
		//获取current的相邻节点
		std::vector<mapnode*> *neighbors =  get_neighbors(current_node);
		if(neighbors)
		{
			int size = neighbors->size();
			for(int i =0 ; i < size; i++)
			{
				mapnode *neighbor = (*neighbors)[i];
				if(neighbor->pre || neighbor->next){
					continue;//在close表中,不做处理
				}

				if(neighbor->index)//在openlist中
				{
					double new_G = current_node->G + cost_2_neighbor(current_node,neighbor);
					if(new_G < neighbor->G)
					{
						//经过当前neighbor路径更佳,更新路径
						neighbor->G = new_G;
						neighbor->F = neighbor->G + neighbor->H;
						neighbor->parent = current_node;
						open_list.change(neighbor);
					}
					continue;
				}
				neighbor->parent = current_node;
				neighbor->G = current_node->G + cost_2_neighbor(current_node,neighbor);
				neighbor->H = cost_2_goal(neighbor,to);
				neighbor->F = neighbor->G + neighbor->H;
				open_list.insert(neighbor);						
			}
			neighbors = NULL;
		}	
	}
}
