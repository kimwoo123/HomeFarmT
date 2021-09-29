const { Sequelize } = require('sequelize');

module.exports.createStore = () => {
  const db = new Sequelize('iot', 'root', 'ssafy', {
    host: '52.79.134.74',
    dialect: 'mysql'
  });

  const users = db.define('users', {
    createdAt: Sequelize.DATE,
    updatedAt: Sequelize.DATE,
    email: Sequelize.STRING,
    profileImage: Sequelize.STRING,
    token: Sequelize.STRING,
  });

  return { db, users };
};
