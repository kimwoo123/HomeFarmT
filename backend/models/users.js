module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  email: { type: DataTypes.STRING, unique: true },
  userid: { type: DataTypes.INTEGER, autoIncrement: true, primaryKey: true },
  hashid: { type: DataTypes.STRING },
  password: { type: DataTypes.STRING, defaultValue: 'none' }
}, {
  timestamps: false
});
