pipeline {
	agent none
	tools {nodejs 'nodejs'}
	environment {
        PATH = "$PATH:/usr/local/bin"
    }
	stages {
		stage('Build') {
			agent {
				docker {
					image 'node:16-alpine'
				}
			}
			steps {
				sh 'docker-compose down --rmi all'
				sh 'docker-compose up -d'
			}
		}
	}
}
