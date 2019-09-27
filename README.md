git init

git add . -A

git commit -m "v1 仓库名"

git tag v1

curl -u caiweiqing1990 https://api.github.com/user/repos -d '{"name":"仓库名"}'

git remote add origin git@github.com:caiweiqing1990/仓库名.git

git push -u origin master

git push origin --tags

git checkout v1

