pipeline {
	agent none
	tools {nodejs 'nodejs'}
	stages {
		stage('Build') {
			agent any
			steps {
				sh 'docker-compose down --rmi all'
				sh 'docker-compose up -d'
			}
		}
	}
}
