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
					sh 'npm install --legacy-peer-deps'
					sh 'npm run build'
				}
			}
		}
		stage('Docker build') {
			agent any
			steps {
				dir ('frontend') {
					sh 'docker build -t iot_front:front .'
				}
			}
		}
		stage ('Docker run') {
			agent any
			steps {
				dir ('frontend') {
					sh 'docker ps -f name=iot_front -q | xargs --no-run-if-empty docker container stop'
					sh 'docker container ls -a -fname=iot_front -q | xargs -r docker container rm'
					sh 'docker run -d --name iot_front -p 8000:80 iot_front:front'
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
