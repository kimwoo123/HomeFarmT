pipeline {
	agent none
	tools {nodejs 'nodejs'}
	stages {
		stage('Build') {
			agent {
				docker {
					image 'node:16-alpine'
				}
			}
			steps {
				dir ('frontend') {
					sh 'rm -f package-lock.json'
					sh 'yarn install'
					sh 'yarn build'
				}
			}
		}
		stage('Docker build') {
			agent any
			steps {
				dir ('frontend') {
					sh 'docker build -t licipe:front .'
				}
			}
		}
		stage ('Docker run') {
			agent any
			steps {
				dir ('frontend') {
					sh 'docker ps -f name=licipe -q | xargs --no-run-if-empty docker container stop'
					sh 'docker container ls -a -fname=licipe -q | xargs -r docker container rm'
					sh 'docker run -d --name licipe -p 8000:80 licipe:front'
				}
			}
		}
	}
	post {
		success {
			echo 'done'
		}
	}
}
