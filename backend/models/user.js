module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  userid: { type: DataTypes.INTEGER, primaryKey: true}, 
  email: DataTypes.STRING
}, {
  timestamps: false
});
