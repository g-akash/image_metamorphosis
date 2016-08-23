#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int main()
{
	int t;
	cin>>t;
	while(t--)
	{
		int n;
		cin>>n;
		vector<int> vec,tested;
		vec.resize(n);
		tested.resize(n);
		int ans=0;
		for(int i=0;i<n;i++)
		{
			cin>>vec[i];
		}
		for(int i=1;i<n;i++)
		{
			if(vec[i]!=vec[i-1])
			{
				if(tested[i-1])
				{
					ans++;
					tested[i]=1;
				}
				else
				{
					ans+=2;
					tested[i-1]=1;
					tested[i]=1;
				}
			}
		}
		cout<<ans<<endl;
	}
}